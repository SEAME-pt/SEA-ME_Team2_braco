// joystick_display.cpp - joystick control of the myCobot 280 with live status
// displays. Same control logic as joystick_all_joints.cpp, plus:
//   * a full-screen terminal display (display.hpp), over SSH or locally
//   * the Waveshare LCD1602 RGB on I2C bus 0 (lcd_display.hpp): angles when
//     idle, "ARM MOVING..." while moving, backlight = speed mode colour
// Either display may be missing; the arm control works without them.
//
// Safety model: step-ahead (bounded-lead) jogging
// ------------------------------------------------
// The arm is never given a far-away goal. On every loop tick the program reads
// the real joint angles and commands a target only a few degrees ahead of them.
// If commands stop arriving, each joint coasts at most the active mode's
// leadMax (jogging) or HOME_LEAD_DEG (homing) and then stops on its own.
//
// Mapping:
//   Left stick  X / Y  joint 1 / joint 2 (base yaw [inverted] / shoulder pitch)
//   Right stick Y / X  joint 4 / joint 5
//   L1 / R1 (6 / 7)    joint 6 (tool rotation), exclusive while held
//   D-pad Y            joint 3 (elbow pitch)
//   A / B              close / open gripper
//   X (click)          home all joints to 0 (click again to abort)
//   Button 9           cycle speed: blue -> green -> purple -> white -> red
//   Ctrl+C             quit (safely stops the arm)
//
// Usage:
//   ./joystick_display           control the arm with the status screen
//   ./joystick_display --debug   plain text output with event/target details
//                                (the status screen is disabled)

#include "display.hpp"
#include "joystick.hpp"
#include "lcd_display.hpp"
#include "mycobot.hpp"

#include <signal.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace {

const char* JOYSTICK_DEVICE = "/dev/input/js0";

// ---- Speed modes (speed units 1..100 = percent) ---------------------------
struct SpeedMode {
    const char* name;
    int maxSpeed;        // jog speed at full stick deflection
    double leadMax;      // step-ahead at full deflection (= worst-case coast)
    int bumperSpeed;     // fixed J6 speed for L1/R1
    uint8_t r, g, b;     // Atom LED colour
};

const std::array<SpeedMode, 5> SPEED_MODES = {{
    {"slow",   12,  5.0,  8,  0,   0,   255},   // blue
    {"medium", 30,  10.0, 15, 0,   255, 0},     // green
    {"50%",    50,  15.0, 22, 128, 0,   255},   // purple
    {"75%",    75,  20.0, 30, 255, 255, 255},   // white
    {"100%",   100, 25.0, 40, 255, 0,   0},     // red
}};
constexpr size_t START_MODE = 1;     // 1 = medium

constexpr double MIN_SPEED = 3.0;
constexpr double STICK_EXPO = 1.8;
constexpr int GRIPPER_SPEED = 50;

constexpr double LEAD_MIN_DEG = 1.0;

constexpr double SLOW_ZONE_DEG = 25.0;
constexpr double MIN_LIMIT_FACTOR = 0.15;
constexpr double AT_LIMIT_DEG = 0.3;

constexpr int HOME_SPEED = 60;
constexpr double HOME_LEAD_DEG = 15.0;
constexpr double HOME_TOLERANCE_DEG = 1.0;
constexpr int HOME_MAX_BAD_READS = 3;

// Stiffer hold (prevents J4 sag while other joints move)
constexpr double HOLD_TOLERANCE_DEG = 0.8;
constexpr double HOLD_RESEND_PERIOD = 0.1;   // seconds
constexpr int HOLD_SPEED = 20;

constexpr int AXIS_DEADZONE = 5000;
constexpr double AXIS_MAX = 32767.0;
constexpr bool BUMPERS_EXCLUSIVE = true;

constexpr auto COMMAND_GAP = 50ms;
constexpr auto LOOP_PERIOD = 20ms;
constexpr int ANGLE_READ_RETRIES = 1;

// While idle, angles are read only for the display, at most this often.
constexpr double IDLE_POLL_PERIOD = 0.25;   // seconds

struct AxisMap {
    int joint;
    int direction;
};

const std::map<int, AxisMap> AXIS_TO_JOINT = {
    {0, {1, -1}},   // left stick X  -> joint 1 (base yaw) [inverted]
    {1, {2, 1}},    // left stick Y  -> joint 2 (shoulder pitch)
    {3, {4, -1}},   // right stick vertical   -> joint 4
    {2, {5, -1}},   // right stick horizontal -> joint 5
    {7, {3, 1}},    // d-pad Y       -> joint 3 (elbow)
};

const std::set<int> TRIGGER_AXES = {4, 5};

constexpr int BUTTON_GRIPPER_CLOSE = 0;   // A
constexpr int BUTTON_GRIPPER_OPEN = 1;    // B
constexpr int BUTTON_HOME = 3;            // X
constexpr int BUTTON_L1 = 6;              // J6 negative
constexpr int BUTTON_R1 = 7;              // J6 positive
constexpr int BUTTON_SPEED_MODE = 9;      // cycle speed mode

constexpr int64_t SPEED_BUTTON_GUARD_MS = 250;

// ---- Helpers ---------------------------------------------------------------

volatile std::sig_atomic_t g_quit = 0;

void onSignal(int) {
    g_quit = 1;
}

struct Motion {
    int direction;
    double speed;
    double lead;
};
using Wanted = std::map<int, Motion>;

struct Shaped {
    double target;
    int speed;
};

void pause(std::chrono::milliseconds d) {
    std::this_thread::sleep_for(d);
}

double nowSeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

double round1(double v) {
    return std::round(v * 10.0) / 10.0;
}

int valueOr0(const std::map<int, int>& m, int key) {
    auto it = m.find(key);
    return it == m.end() ? 0 : it->second;
}

// printf-style formatting into a std::string (always called with arguments)
template <typename... Args>
std::string format(const char* fmt, Args... args) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), fmt, args...);
    return buf;
}

std::string fmtAngles(const std::optional<mycobot::Angles>& angles, const char* numFormat = "%.2f") {
    if (!angles) return "None";
    std::string out = "[";
    for (size_t i = 0; i < angles->size(); ++i) {
        out += (i ? ", " : "") + format(numFormat, (*angles)[i]);
    }
    return out + "]";
}

std::string fmtJoints(const std::set<int>& joints) {
    std::string out = "[";
    bool first = true;
    for (int j : joints) {
        out += (first ? "" : ", ") + std::to_string(j);
        first = false;
    }
    return out + "]";
}

double stickRatio(int raw) {
    const int magnitude = std::abs(raw);
    if (magnitude <= AXIS_DEADZONE) return 0.0;
    const double ratio = std::min(1.0, (magnitude - AXIS_DEADZONE) / (AXIS_MAX - AXIS_DEADZONE));
    return std::pow(ratio, STICK_EXPO);
}

Motion motionForRatio(double ratio, const SpeedMode& mode, int direction) {
    return {direction,
            MIN_SPEED + ratio * (mode.maxSpeed - MIN_SPEED),
            LEAD_MIN_DEG + ratio * (mode.leadMax - LEAD_MIN_DEG)};
}

double bumperRatio(const SpeedMode& mode) {
    const double span = mode.maxSpeed - MIN_SPEED;
    return std::clamp((mode.bumperSpeed - MIN_SPEED) / span, 0.0, 1.0);
}

std::optional<Shaped> shapeNearLimit(const mycobot::JointLimit& lim, double angle,
                                     int direction, double speed, double lead) {
    const double remaining = direction > 0 ? (lim.max - angle) : (angle - lim.min);
    if (remaining <= AT_LIMIT_DEG) return std::nullopt;

    const double factor = remaining >= SLOW_ZONE_DEG
                              ? 1.0
                              : std::max(MIN_LIMIT_FACTOR, remaining / SLOW_ZONE_DEG);
    const int shapedSpeed = std::max(1, static_cast<int>(std::lround(speed * factor)));
    lead = std::min(lead * factor, remaining);
    return Shaped{angle + direction * lead, shapedSpeed};
}

Wanted wantedMotion(const std::map<int, int>& axisState,
                    const std::map<int, int>& buttonState,
                    const SpeedMode& mode,
                    const std::set<int>& blocked) {
    Wanted wanted;

    const bool l1 = valueOr0(buttonState, BUTTON_L1) == 1;
    const bool r1 = valueOr0(buttonState, BUTTON_R1) == 1;
    const int bumperDir = (l1 && !r1) ? -1 : ((r1 && !l1) ? 1 : 0);
    if (bumperDir != 0) {
        wanted[6] = motionForRatio(bumperRatio(mode), mode, bumperDir);
        if (BUMPERS_EXCLUSIVE) return wanted;
    }

    for (const auto& [axis, map] : AXIS_TO_JOINT) {
        if (blocked.count(axis)) continue;
        const int value = valueOr0(axisState, axis);
        const double ratio = stickRatio(value);
        if (ratio > 0.0) {
            const int sign = value > 0 ? map.direction : -map.direction;
            wanted[map.joint] = motionForRatio(ratio, mode, sign);
        }
    }
    return wanted;
}

void setLed(mycobot::MyCobot280& mc, const SpeedMode& mode, ui::TerminalDisplay& display) {
    try {
        mc.setColor(mode.r, mode.g, mode.b);
    } catch (const std::exception& e) {
        display.addMessage(format("!!! could not set LED colour (%s)", e.what()));
    }
    pause(COMMAND_GAP);
}

// ---- Main control loop -----------------------------------------------------

// `status` belongs to the caller, so the last state is still available after
// the loop ends (used to show the final angles on the LCD).
void controlLoop(input::Joystick& js, mycobot::MyCobot280& mc,
                 ui::TerminalDisplay& display, ui::LcdDisplay& lcd,
                 ui::Status& status, bool debug) {
    size_t modeIndex = START_MODE;
    const SpeedMode* mode = &SPEED_MODES[modeIndex];
    setLed(mc, *mode, display);
    display.addMessage(format("Ready. Speed mode: %s (cap %d %%)", mode->name, mode->maxSpeed));

    std::map<int, int> axisState;
    std::map<int, int> buttonState;
    std::set<int> active;
    std::map<int, double> hold;
    std::map<int, double> lastHoldSend;
    std::set<int> limitReported;
    bool homing = false;
    int badReads = 0;
    std::map<int, int64_t> axisChangedMs;
    std::set<int> blockedAxes;
    std::optional<int64_t> speedBtnMs;

    // Display state
    for (int j = 0; j < 6; ++j) {
        const auto& lim = mc.softLimits()[j];
        status.limits[j] = {lim.min, lim.max};
    }
    double lastIdlePoll = 0.0;

    auto debugMsg = [&](const std::string& text) {
        if (debug) display.addMessage(text);
    };

    auto stopArm = [&]() {
        mc.stop();
        pause(COMMAND_GAP);
        active.clear();
        hold.clear();
    };

    // Store a fresh angle reading for the display
    auto recordAngles = [&](const std::optional<mycobot::Angles>& angles) {
        status.feedbackOk = angles.has_value();
        if (angles) status.angles = *angles;
    };

    // One loop tick. Returns early where the Python version used `continue`.
    auto tick = [&]() {
        // 1. Drain joystick events
        bool homeClicked = false;
        for (const input::Event& ev : js.readEvents()) {
            const int64_t tMs = ev.timeMs;
            const int number = ev.number;
            const int value = ev.value;

            if (ev.type == input::TYPE_AXIS) {
                if (TRIGGER_AXES.count(number)) continue;
                axisState[number] = value;
                axisChangedMs[number] = tMs;
                const bool nearSpeedBtn =
                    valueOr0(buttonState, BUTTON_SPEED_MODE) == 1 ||
                    (speedBtnMs && std::llabs(tMs - *speedBtnMs) <= SPEED_BUTTON_GUARD_MS);
                if (nearSpeedBtn && std::abs(value) > AXIS_DEADZONE) {
                    if (!blockedAxes.count(number)) {
                        debugMsg(format("axis   %d blocked (fired with button 9)", number));
                    }
                    blockedAxes.insert(number);
                } else if (blockedAxes.count(number) && std::abs(value) <= AXIS_DEADZONE) {
                    blockedAxes.erase(number);
                    debugMsg(format("axis   %d unblocked", number));
                }
                debugMsg(format("axis   %d = %d", number, value));

            } else if (ev.type == input::TYPE_BUTTON) {
                buttonState[number] = value;
                debugMsg(format("button %d = %d", number, value));

                if (number == BUTTON_SPEED_MODE) {
                    speedBtnMs = tMs;
                    for (const auto& [axis, axisMs] : axisChangedMs) {
                        if (tMs - axisMs <= SPEED_BUTTON_GUARD_MS &&
                            std::abs(valueOr0(axisState, axis)) > AXIS_DEADZONE) {
                            blockedAxes.insert(axis);
                        }
                    }
                }

                if (value == 1) {
                    if (number == BUTTON_GRIPPER_CLOSE) {
                        display.addMessage(">>> close gripper");
                        mc.setGripperState(1, GRIPPER_SPEED);
                    } else if (number == BUTTON_GRIPPER_OPEN) {
                        display.addMessage(">>> open gripper");
                        mc.setGripperState(0, GRIPPER_SPEED);
                    } else if (number == BUTTON_HOME) {
                        homeClicked = true;
                    } else if (number == BUTTON_SPEED_MODE) {
                        modeIndex = (modeIndex + 1) % SPEED_MODES.size();
                        mode = &SPEED_MODES[modeIndex];
                        setLed(mc, *mode, display);
                        display.addMessage(format(">>> speed mode: %s (cap %d %%)",
                                                  mode->name, mode->maxSpeed));
                    }
                }
            }
        }

        const Wanted wanted = wantedMotion(axisState, buttonState, *mode, blockedAxes);

        // 2. Homing state transitions
        if (homeClicked) {
            if (homing) {
                display.addMessage(">>> homing cancelled");
                homing = false;
                stopArm();
            } else {
                display.addMessage(">>> homing (click X again or touch a stick to abort)");
                homing = true;
                badReads = 0;
                active.clear();
                hold.clear();
            }
        }
        if (homing && !wanted.empty()) {
            display.addMessage(">>> homing aborted by stick/bumper input");
            homing = false;
            stopArm();
        }

        // 3. Homing: bounded, synchronised steps toward all-zero
        if (homing) {
            const auto angles = mc.getAngles(ANGLE_READ_RETRIES);
            recordAngles(angles);
            if (!angles) {
                ++badReads;
                mc.stop();
                pause(COMMAND_GAP);
                if (badReads >= HOME_MAX_BAD_READS) {
                    display.addMessage("!!! no valid angle feedback, homing aborted");
                    homing = false;
                }
                return;
            }
            badReads = 0;

            double maxDev = 0.0;
            for (double a : *angles) maxDev = std::max(maxDev, std::abs(a));

            if (maxDev <= HOME_TOLERANCE_DEG) {
                display.addMessage(">>> home reached");
                homing = false;
            } else {
                const double scale = std::min(1.0, HOME_LEAD_DEG / maxDev);
                mycobot::Angles targets{};
                for (size_t i = 0; i < targets.size(); ++i) {
                    targets[i] = round1((*angles)[i] * (1.0 - scale));
                }
                debugMsg("home targets " + fmtAngles(targets, "%.1f"));
                mc.sendAngles(targets, HOME_SPEED);
                pause(COMMAND_GAP);
            }
            return;
        }

        // 4. Nothing requested -> stop once, then idle
        if (wanted.empty()) {
            if (!active.empty()) {
                display.addMessage(">>> stop");
                stopArm();
            }
            hold.clear();
            limitReported.clear();

            // Read angles occasionally for the display only. A failed read
            // here moves nothing, it only shows "no feedback".
            const double now = nowSeconds();
            if ((display.active() || lcd.ok()) && now - lastIdlePoll >= IDLE_POLL_PERIOD) {
                lastIdlePoll = now;
                recordAngles(mc.getAngles(ANGLE_READ_RETRIES));
            }
            return;
        }

        // 5. Jogging requires fresh feedback
        const auto angles = mc.getAngles(ANGLE_READ_RETRIES);
        recordAngles(angles);
        if (!angles) {
            display.addMessage("!!! no valid angle feedback, stopping");
            stopArm();
            return;
        }

        const double now = nowSeconds();
        std::set<int> moving;
        for (const auto& [joint, motion] : wanted) moving.insert(joint);

        std::set<int> released;
        std::set_difference(active.begin(), active.end(), moving.begin(), moving.end(),
                            std::inserter(released, released.begin()));

        std::set<int> stillReported;
        std::set_intersection(limitReported.begin(), limitReported.end(),
                              moving.begin(), moving.end(),
                              std::inserter(stillReported, stillReported.begin()));
        limitReported = stillReported;

        // 6a. Latch idle joints
        for (int j = 1; j <= 6; ++j) {
            if (moving.count(j)) {
                hold.erase(j);
            } else if (!hold.count(j)) {
                hold[j] = (*angles)[j - 1];
            }
        }

        // 6b. Released joints: pin at the latched angle
        for (int j : released) {
            mc.sendAngle(j, round1(hold[j]), HOLD_SPEED);
            lastHoldSend[j] = now;
            pause(COMMAND_GAP);
        }

        // 6c. Idle joints that drifted
        for (const auto& [j, held] : hold) {
            if (released.count(j)) continue;
            const double drift = (*angles)[j - 1] - held;
            auto it = lastHoldSend.find(j);
            const double lastSend = it == lastHoldSend.end() ? 0.0 : it->second;
            if (std::abs(drift) > HOLD_TOLERANCE_DEG && now - lastSend >= HOLD_RESEND_PERIOD) {
                debugMsg(format("hold J%d: drift %+.1f, back to %.1f", j, drift, held));
                mc.sendAngle(j, round1(held), HOLD_SPEED);
                lastHoldSend[j] = now;
                pause(COMMAND_GAP);
            }
        }

        // 6d. Moving joints: proportional speed, slowed near limits
        for (const auto& [joint, motion] : wanted) {
            const double angle = (*angles)[joint - 1];
            const auto shaped = shapeNearLimit(mc.softLimits()[joint - 1], angle,
                                               motion.direction, motion.speed, motion.lead);
            if (!shaped) {
                if (!limitReported.count(joint)) {
                    display.addMessage(format(">>> J%d at limit", joint));
                    limitReported.insert(joint);
                }
                continue;
            }
            limitReported.erase(joint);
            debugMsg(format("J%d %.1f -> %.1f @%d", joint, angle, shaped->target, shaped->speed));
            mc.sendAngle(joint, round1(shaped->target), shaped->speed);
            pause(COMMAND_GAP);
        }

        if (moving != active) {
            display.addMessage(">>> jogging joints " + fmtJoints(moving));
        }
        active = moving;
    };

    while (!g_quit) {
        tick();

        // Update the screen (rate-limited inside the display class)
        status.modeName = mode->name;
        status.speedCap = mode->maxSpeed;
        status.maxCoast = mode->leadMax;
        status.r = mode->r;
        status.g = mode->g;
        status.b = mode->b;
        status.homing = homing;
        for (int j = 1; j <= 6; ++j) {
            status.moving[j - 1] = active.count(j) > 0;
            status.atLimit[j - 1] = limitReported.count(j) > 0;
        }
        display.show(status);
        lcd.show(status);

        pause(LOOP_PERIOD);
    }
}

}  // namespace

int main(int argc, char** argv) {
    bool debug = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--debug") == 0) debug = true;
    }

    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    struct sigaction sa {};
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGHUP, &sa, nullptr);

    try {
        std::printf("1/3 Initialising LCD1602 RGB on %s\n", ui::LcdDisplay::DEFAULT_BUS);
        ui::LcdDisplay lcd;
        if (lcd.ok()) {
            std::printf("    OK\n");
        } else {
            std::printf("!!! LCD not available (%s) - continuing without it\n",
                        lcd.error().c_str());
        }

        std::printf("2/3 Opening joystick: %s\n", JOYSTICK_DEVICE);
        input::Joystick js(JOYSTICK_DEVICE);
        std::printf("    OK\n");

        std::printf("3/3 Connecting to arm on: %s\n", mycobot::MyCobot280::DEFAULT_PORT);
        mycobot::MyCobot280 mc;
        pause(1000ms);
        mc.powerOn();
        pause(500ms);
        std::printf("    Current angles: %s\n", fmtAngles(mc.getAngles()).c_str());

        int exitCode = 0;
        std::string endReason = "Quitting.";
        ui::Status status;
        std::string stopError;

        {
            // The status screen exists only inside this block; leaving it
            // restores the normal terminal so the final messages stay visible.
            ui::TerminalDisplay display(!debug);
            try {
                controlLoop(js, mc, display, lcd, status, debug);
            } catch (const std::exception& e) {
                endReason = std::string("Stopping: ") + e.what();   // e.g. joystick unplugged
                exitCode = 1;
            }

            // Stop the arm first, before anything else
            try {
                mc.stop();
            } catch (const std::exception& e) {
                stopError = e.what();
                exitCode = 1;
            }
        }

        std::printf("\n%s\n", endReason.c_str());
        if (!stopError.empty()) {
            std::fprintf(stderr, "!!! could not send final stop: %s\n", stopError.c_str());
        }
        pause(200ms);
        const auto finalAngles = mc.getAngles();
        std::printf("Arm stopped. Final angles: %s\n", fmtAngles(finalAngles).c_str());

        // Leave the final angles on the LCD (backlight keeps the mode colour)
        status.moving = {};
        status.homing = false;
        status.feedbackOk = finalAngles.has_value();
        if (finalAngles) status.angles = *finalAngles;
        lcd.show(status, true);
        return exitCode;

    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
}

