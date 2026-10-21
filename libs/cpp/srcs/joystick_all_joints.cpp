// joystick_all_joints.cpp - joystick control of all 6 joints of the myCobot 280.
// C++ version of joystick_all_joints.py, using mycobot.hpp and joystick.hpp.
//
// Safety model: step-ahead (bounded-lead) jogging
// ------------------------------------------------
// The arm is never given a far-away goal. On every loop tick the program reads
// the real joint angles and commands a target only a few degrees ahead of them.
// If commands stop arriving (crash, kill -9, frozen process, lost serial data,
// joystick unplugged), each joint coasts at most the active mode's leadMax
// (jogging) or HOME_LEAD_DEG (homing) and then stops on its own.
//
// Behaviour:
//   * Stick speed is proportional to deflection (expo curve near the centre).
//   * Five speed modes, cycled with button 9. The Atom LED shows the mode:
//     blue = slow (12%), green = medium (30%), purple = 50%, white = 75%,
//     red = 100%.
//   * Near a joint limit, speed and step shrink with the remaining distance.
//   * Idle joints are latched and held while other joints jog.
//   * Homing is one click: X (button 3) walks all joints to 0. Click X again,
//     or touch any stick / bumper, to abort.
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
//   ./joystick_all_joints           control the arm
//   ./joystick_all_joints --debug   print events, targets, hold corrections

#include "joystick.hpp"
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
// maxSpeed:    jog speed at full stick deflection
// leadMax:     step-ahead at full deflection (= worst-case coast)
// bumperSpeed: fixed J6 speed for L1/R1
// r, g, b:     Atom LED colour
struct SpeedMode {
    const char* name;
    int maxSpeed;
    double leadMax;
    int bumperSpeed;
    uint8_t r, g, b;
};

const std::array<SpeedMode, 5> SPEED_MODES = {{
    {"12%",   12,  5.0,  10, 0,   0,   255},   // blue
    {"30%",   30,  10.0, 20, 0,   255, 0},     // green
    {"50%",   50,  15.0, 30, 128, 0,   255},   // purple
    {"75%",   75,  20.0, 40, 255, 255, 255},   // white
    {"100%",  100, 25.0, 50, 255, 0,   0},     // red
}};
constexpr size_t START_MODE = 1;     // 1 = medium

constexpr double MIN_SPEED = 3.0;    // speed just outside the deadzone
constexpr double STICK_EXPO = 1.8;   // >1 = more resolution near centre
constexpr int GRIPPER_SPEED = 50;

// ---- Step-ahead lead -------------------------------------------------------
constexpr double LEAD_MIN_DEG = 1.0;

// ---- Soft limit zone -------------------------------------------------------
constexpr double SLOW_ZONE_DEG = 25.0;
constexpr double MIN_LIMIT_FACTOR = 0.15;
constexpr double AT_LIMIT_DEG = 0.3;

// ---- One-click homing ------------------------------------------------------
constexpr int HOME_SPEED = 60;
constexpr double HOME_LEAD_DEG = 15.0;
constexpr double HOME_TOLERANCE_DEG = 1.0;
constexpr int HOME_MAX_BAD_READS = 3;

// ---- Idle-joint hold -------------------------------------------------------
constexpr double HOLD_TOLERANCE_DEG = 1.5;
constexpr double HOLD_RESEND_PERIOD = 0.3;   // seconds
constexpr int HOLD_SPEED = 10;

// ---- Input -----------------------------------------------------------------
constexpr int AXIS_DEADZONE = 5000;
constexpr double AXIS_MAX = 32767.0;
constexpr bool BUMPERS_EXCLUSIVE = true;

constexpr auto COMMAND_GAP = 50ms;
constexpr auto LOOP_PERIOD = 20ms;

// One attempt per angle read while moving: a read that fails is not retried
// while a joint may still be moving.
constexpr int ANGLE_READ_RETRIES = 1;

// A missed read only skips that tick: the arm keeps heading for the target of
// the last tick (at most the mode's lead away). The arm is stopped only when
// there has been no good reading for this long, the same limit as the
// driver's command watchdog in the ROS 2 system (command_timeout_ms).
constexpr double STATE_GRACE_S = 0.3;

struct AxisMap {
    int joint;
    int direction;
};

// axis number -> (joint 1..6, direction)
const std::map<int, AxisMap> AXIS_TO_JOINT = {
    {0, {1, -1}},   // left stick X  -> joint 1 (base yaw) [inverted]
    {1, {2, 1}},    // left stick Y  -> joint 2 (shoulder pitch)
    {3, {4, -1}},   // right stick vertical   -> joint 4
    {2, {5, -1}},   // right stick horizontal -> joint 5
    {7, {3, 1}},    // d-pad Y       -> joint 3 (elbow)
};

// Analog trigger axes: ignored completely, never move a joint.
const std::set<int> TRIGGER_AXES = {4, 5};

constexpr int BUTTON_GRIPPER_CLOSE = 0;   // A
constexpr int BUTTON_GRIPPER_OPEN = 1;    // B
constexpr int BUTTON_HOME = 3;            // X
constexpr int BUTTON_L1 = 6;              // J6 negative
constexpr int BUTTON_R1 = 7;              // J6 positive
constexpr int BUTTON_SPEED_MODE = 9;      // cycle speed mode

// Axes that change while button 9 is held, or within this window of its
// press/release, are ignored until they return to centre.
constexpr int64_t SPEED_BUTTON_GUARD_MS = 250;

// Joint limits come from the library: mycobot::JOINT_LIMITS (mycobot.hpp).

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
using Wanted = std::map<int, Motion>;   // joint -> motion (sorted by joint)

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

std::string fmtAngles(const std::optional<mycobot::Angles>& angles, const char* format = "%.2f") {
    if (!angles) return "None";
    std::string out = "[";
    char buf[32];
    for (size_t i = 0; i < angles->size(); ++i) {
        std::snprintf(buf, sizeof(buf), format, (*angles)[i]);
        out += (i ? ", " : "");
        out += buf;
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

// Deflection beyond the deadzone, 0..1, shaped by the expo curve.
double stickRatio(int raw) {
    const int magnitude = std::abs(raw);
    if (magnitude <= AXIS_DEADZONE) return 0.0;
    const double ratio = std::min(1.0, (magnitude - AXIS_DEADZONE) / (AXIS_MAX - AXIS_DEADZONE));
    return std::pow(ratio, STICK_EXPO);
}

// Map a 0..1 command ratio to (speed, lead) for the given mode.
Motion motionForRatio(double ratio, const SpeedMode& mode, int direction) {
    return {direction,
            MIN_SPEED + ratio * (mode.maxSpeed - MIN_SPEED),
            LEAD_MIN_DEG + ratio * (mode.leadMax - LEAD_MIN_DEG)};
}

// Ratio that yields the mode's fixed J6 bumper speed.
double bumperRatio(const SpeedMode& mode) {
    const double span = mode.maxSpeed - MIN_SPEED;
    return std::clamp((mode.bumperSpeed - MIN_SPEED) / span, 0.0, 1.0);
}

// Scale speed and lead by the distance left to the limit we move toward.
// Returns nothing if the joint is already at the limit.
std::optional<Shaped> shapeNearLimit(const mycobot::JointLimit& lim, double angle,
                                     int direction, double speed, double lead) {
    const double remaining = direction > 0 ? (lim.max - angle) : (angle - lim.min);
    if (remaining <= AT_LIMIT_DEG) return std::nullopt;

    const double factor = remaining >= SLOW_ZONE_DEG
                              ? 1.0
                              : std::max(MIN_LIMIT_FACTOR, remaining / SLOW_ZONE_DEG);
    const int shapedSpeed = std::max(1, static_cast<int>(std::lround(speed * factor)));
    lead = std::min(lead * factor, remaining);   // never step past the limit
    return Shaped{angle + direction * lead, shapedSpeed};
}

// Desired motion per joint. Axes in `blocked` never produce motion.
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
        if (BUMPERS_EXCLUSIVE) return wanted;   // ignore sticks while J6 rotates
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

void setLed(mycobot::MyCobot280& mc, const SpeedMode& mode) {
    try {
        mc.setColor(mode.r, mode.g, mode.b);
    } catch (const std::exception& e) {
        std::printf("!!! could not set LED colour (%s)\n", e.what());
    }
    pause(COMMAND_GAP);
}

void printMode(const char* prefix, const SpeedMode& mode) {
    std::printf("%s%s (cap %d %%, max coast %.0f deg)\n",
                prefix, mode.name, mode.maxSpeed, mode.leadMax);
}

// ---- Main control loop -----------------------------------------------------

void controlLoop(input::Joystick& js, mycobot::MyCobot280& mc, bool debug) {
    size_t modeIndex = START_MODE;
    const SpeedMode* mode = &SPEED_MODES[modeIndex];
    setLed(mc, *mode);
    std::printf("Ready. Speed mode: %s (cap %d %%, max coast %.0f deg). "
                "Button 9 changes speed, X homes, Ctrl+C quits.\n",
                mode->name, mode->maxSpeed, mode->leadMax);

    std::map<int, int> axisState;
    std::map<int, int> buttonState;
    std::set<int> active;                  // joints commanded on the previous tick
    std::map<int, double> hold;            // idle joint -> latched angle
    std::map<int, double> lastHoldSend;    // joint -> time of last hold command
    std::set<int> limitReported;           // joints whose "at limit" was printed
    bool homing = false;
    int badReads = 0;
    std::map<int, int64_t> axisChangedMs;  // axis -> joystick time of last event
    std::set<int> blockedAxes;             // axes that fired with button 9
    std::optional<int64_t> speedBtnMs;     // joystick time of last button 9 event

    auto stopArm = [&]() {
        mc.stop();
        pause(COMMAND_GAP);
        active.clear();
        hold.clear();
    };

    // Feedback for one tick: Fresh (angles filled in), Missed (no reply, but
    // the last good reading is recent: skip the tick) or Lost (stop the arm).
    enum class Feedback { Fresh, Missed, Lost };
    double lastGoodRead = -1e9;
    auto readFeedback = [&](mycobot::Angles& out) {
        const auto angles = mc.getAngles(ANGLE_READ_RETRIES);
        const double now = nowSeconds();
        if (angles) {
            out = *angles;
            lastGoodRead = now;
            return Feedback::Fresh;
        }
        return now - lastGoodRead <= STATE_GRACE_S ? Feedback::Missed : Feedback::Lost;
    };

    while (!g_quit) {
        // 1. Drain joystick events
        bool homeClicked = false;
        for (const input::Event& ev : js.readEvents()) {
            const int64_t tMs = ev.timeMs;
            const int number = ev.number;
            const int value = ev.value;

            if (ev.type == input::TYPE_AXIS) {
                if (TRIGGER_AXES.count(number)) continue;   // never a joint command
                axisState[number] = value;
                axisChangedMs[number] = tMs;
                const bool nearSpeedBtn =
                    valueOr0(buttonState, BUTTON_SPEED_MODE) == 1 ||
                    (speedBtnMs && std::llabs(tMs - *speedBtnMs) <= SPEED_BUTTON_GUARD_MS);
                if (nearSpeedBtn && std::abs(value) > AXIS_DEADZONE) {
                    if (!blockedAxes.count(number) && debug) {
                        std::printf("axis   %d blocked (fired with button 9)\n", number);
                    }
                    blockedAxes.insert(number);
                } else if (blockedAxes.count(number) && std::abs(value) <= AXIS_DEADZONE) {
                    blockedAxes.erase(number);   // back at centre -> usable
                    if (debug) std::printf("axis   %d unblocked\n", number);
                }
                if (debug) std::printf("axis   %d = %d\n", number, value);

            } else if (ev.type == input::TYPE_BUTTON) {
                buttonState[number] = value;
                if (debug) std::printf("button %d = %d\n", number, value);

                if (number == BUTTON_SPEED_MODE) {
                    speedBtnMs = tMs;
                    // Block axes that moved just before button 9 fired
                    for (const auto& [axis, axisMs] : axisChangedMs) {
                        if (tMs - axisMs <= SPEED_BUTTON_GUARD_MS &&
                            std::abs(valueOr0(axisState, axis)) > AXIS_DEADZONE) {
                            blockedAxes.insert(axis);
                        }
                    }
                }

                if (value == 1) {
                    if (number == BUTTON_GRIPPER_CLOSE) {
                        std::printf(">>> close gripper\n");
                        mc.setGripperState(1, GRIPPER_SPEED);
                    } else if (number == BUTTON_GRIPPER_OPEN) {
                        std::printf(">>> open gripper\n");
                        mc.setGripperState(0, GRIPPER_SPEED);
                    } else if (number == BUTTON_HOME) {
                        homeClicked = true;
                    } else if (number == BUTTON_SPEED_MODE) {
                        modeIndex = (modeIndex + 1) % SPEED_MODES.size();
                        mode = &SPEED_MODES[modeIndex];
                        setLed(mc, *mode);
                        printMode(">>> speed mode: ", *mode);
                    }
                }
            }
        }

        const Wanted wanted = wantedMotion(axisState, buttonState, *mode, blockedAxes);

        // 2. Homing state transitions (click toggles, any jog input aborts)
        if (homeClicked) {
            if (homing) {
                std::printf(">>> homing cancelled\n");
                homing = false;
                stopArm();
            } else {
                std::printf(">>> homing (click X again or touch a stick to abort)\n");
                homing = true;
                badReads = 0;
                active.clear();
                hold.clear();
            }
        }
        if (homing && !wanted.empty()) {
            std::printf(">>> homing aborted by stick/bumper input\n");
            homing = false;
            stopArm();
        }

        // 3. Homing: bounded, synchronised steps toward all-zero
        if (homing) {
            mycobot::Angles angles{};
            const Feedback feedback = readFeedback(angles);
            if (feedback == Feedback::Missed) {
                if (debug) std::printf("angle read missed, skipping the tick\n");
                pause(LOOP_PERIOD);
                continue;
            }
            if (feedback == Feedback::Lost) {
                ++badReads;
                mc.stop();
                pause(COMMAND_GAP);
                if (badReads >= HOME_MAX_BAD_READS) {
                    std::printf("!!! no valid angle feedback, homing aborted\n");
                    homing = false;
                }
                pause(LOOP_PERIOD);
                continue;
            }
            badReads = 0;

            double maxDev = 0.0;
            for (double a : angles) maxDev = std::max(maxDev, std::abs(a));

            if (maxDev <= HOME_TOLERANCE_DEG) {
                std::printf(">>> home reached\n");
                homing = false;
            } else {
                // Same fraction for every joint -> they all arrive together
                const double scale = std::min(1.0, HOME_LEAD_DEG / maxDev);
                mycobot::Angles targets{};
                for (size_t i = 0; i < targets.size(); ++i) {
                    targets[i] = round1(angles[i] * (1.0 - scale));
                }
                if (debug) std::printf("home targets %s\n", fmtAngles(targets, "%.1f").c_str());
                mc.sendAngles(targets, HOME_SPEED);
                pause(COMMAND_GAP);
            }
            pause(LOOP_PERIOD);
            continue;
        }

        // 4. Nothing requested -> stop once, then idle (no serial traffic)
        if (wanted.empty()) {
            if (!active.empty()) {
                std::printf(">>> stop\n");
                stopArm();
            }
            hold.clear();
            limitReported.clear();
            pause(LOOP_PERIOD);
            continue;
        }

        // 5. Jogging requires fresh feedback
        mycobot::Angles angles{};
        const Feedback feedback = readFeedback(angles);
        if (feedback == Feedback::Missed) {
            if (debug) std::printf("angle read missed, skipping the tick\n");
            pause(LOOP_PERIOD);
            continue;
        }
        if (feedback == Feedback::Lost) {
            std::printf("!!! no valid angle feedback, stopping\n");
            stopArm();
            pause(LOOP_PERIOD);
            continue;
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

        // 6a. Latch idle joints (at session start or on release)
        for (int j = 1; j <= 6; ++j) {
            if (moving.count(j)) {
                hold.erase(j);
            } else if (!hold.count(j)) {
                hold[j] = angles[j - 1];
            }
        }

        // 6b. Released joints: pin at the latched angle
        for (int j : released) {
            mc.sendAngle(j, round1(hold[j]), HOLD_SPEED);
            lastHoldSend[j] = now;
            pause(COMMAND_GAP);
        }

        // 6c. Idle joints that drifted (e.g. J4 sag while J6 rotates)
        for (const auto& [j, held] : hold) {
            if (released.count(j)) continue;
            const double drift = angles[j - 1] - held;
            auto it = lastHoldSend.find(j);
            const double lastSend = it == lastHoldSend.end() ? 0.0 : it->second;
            if (std::abs(drift) > HOLD_TOLERANCE_DEG && now - lastSend >= HOLD_RESEND_PERIOD) {
                if (debug) {
                    std::printf("hold J%d: drift %+.1f, back to %.1f\n", j, drift, held);
                }
                mc.sendAngle(j, round1(held), HOLD_SPEED);
                lastHoldSend[j] = now;
                pause(COMMAND_GAP);
            }
        }

        // 6d. Moving joints: proportional speed, slowed near limits
        for (const auto& [joint, motion] : wanted) {
            const double angle = angles[joint - 1];
            const auto shaped = shapeNearLimit(mycobot::JOINT_LIMITS[joint - 1], angle,
                                               motion.direction, motion.speed, motion.lead);
            if (!shaped) {
                if (!limitReported.count(joint)) {
                    std::printf(">>> J%d at limit\n", joint);
                    limitReported.insert(joint);
                }
                continue;
            }
            limitReported.erase(joint);
            if (debug) {
                std::printf("J%d %.1f -> %.1f @%d\n", joint, angle, shaped->target, shaped->speed);
            }
            mc.sendAngle(joint, round1(shaped->target), shaped->speed);
            pause(COMMAND_GAP);
        }

        if (moving != active) {
            std::printf(">>> jogging joints %s\n", fmtJoints(moving).c_str());
        }
        active = moving;
        pause(LOOP_PERIOD);
    }
}

}  // namespace

int main(int argc, char** argv) {
    bool debug = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--debug") == 0) debug = true;
    }

    // Line-buffered output, so messages appear immediately (also under systemd)
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    // Ctrl+C, kill and terminal hang-up all end the loop and stop the arm
    struct sigaction sa {};
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGHUP, &sa, nullptr);

    try {
        std::printf("1/2 Opening joystick: %s\n", JOYSTICK_DEVICE);
        input::Joystick js(JOYSTICK_DEVICE);
        std::printf("    OK\n");

        std::printf("2/2 Connecting to arm on: %s\n", mycobot::MyCobot280::DEFAULT_PORT);
        mycobot::MyCobot280 mc;
        pause(1000ms);
        mc.powerOn();
        pause(500ms);
        std::printf("    Current angles: %s\n", fmtAngles(mc.getAngles()).c_str());

        int exitCode = 0;
        try {
            controlLoop(js, mc, debug);
            std::printf("\nQuitting.\n");
        } catch (const std::exception& e) {
            std::printf("\nStopping: %s\n", e.what());   // e.g. joystick unplugged
            exitCode = 1;
        }

        // Always stop the arm, whatever ended the loop
        try {
            mc.stop();
            pause(200ms);
            std::printf("Arm stopped. Final angles: %s\n", fmtAngles(mc.getAngles()).c_str());
        } catch (const std::exception& e) {
            std::fprintf(stderr, "!!! could not send final stop: %s\n", e.what());
            exitCode = 1;
        }
        return exitCode;

    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
}
