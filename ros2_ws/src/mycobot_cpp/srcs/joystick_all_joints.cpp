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
//     blue = slow (15%), green = medium (30%), purple = 50%, white = 75%,
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

#include "joystick.hpp"
#include "mycobot.hpp"
#include "teleop_logic.hpp"

using namespace std::chrono_literals;
using namespace mycobot::teleop;

namespace {

const char* JOYSTICK_DEVICE = "/dev/input/js0";

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

// Joint limits come from the library: mycobot::JOINT_LIMITS (mycobot.hpp).

// ---- Helpers ---------------------------------------------------------------

volatile std::sig_atomic_t g_quit = 0;

void onSignal(int) { g_quit = 1; }

void pause(std::chrono::milliseconds d) { std::this_thread::sleep_for(d); }

double nowSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
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

void setLed(mycobot::MyCobot280& mc, const SpeedMode& mode) {
    try {
        mc.setColor(mode.r, mode.g, mode.b);
    } catch (const std::exception& e) {
        std::printf("!!! could not set LED colour (%s)\n", e.what());
    }
    pause(COMMAND_GAP);
}

void printMode(const char* prefix, const SpeedMode& mode) {
    std::printf("%s%s (cap %d %%, max coast %.0f deg)\n", prefix, mode.name, mode.maxSpeed,
                mode.leadMax);
}

// ---- Main control loop -----------------------------------------------------

void controlLoop(input::Joystick& js, mycobot::MyCobot280& mc, bool debug) {
    size_t modeIndex = START_MODE;
    const SpeedMode* mode = &SPEED_MODES[modeIndex];
    setLed(mc, *mode);
    std::printf(
        "Ready. Speed mode: %s (cap %d %%, max coast %.0f deg). "
        "Button 9 changes speed, X homes, Ctrl+C quits.\n",
        mode->name, mode->maxSpeed, mode->leadMax);

    std::map<int, int> axisState;
    std::map<int, int> buttonState;
    std::set<int> active;                // joints commanded on the previous tick
    std::map<int, double> hold;          // idle joint -> latched angle
    std::map<int, double> lastHoldSend;  // joint -> time of last hold command
    std::set<int> limitReported;         // joints whose "at limit" was printed
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
                if (isTriggerAxis(number)) continue;  // never a joint command
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
                    blockedAxes.erase(number);  // back at centre -> usable
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

            // Same fraction for every joint -> they all arrive together
            if (const auto targets = homeStep(angles)) {
                if (debug) std::printf("home targets %s\n", fmtAngles(*targets, "%.1f").c_str());
                mc.sendAngles(*targets, HOME_SPEED);
                pause(COMMAND_GAP);
            } else {
                std::printf(">>> home reached\n");
                homing = false;
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
        std::set_intersection(limitReported.begin(), limitReported.end(), moving.begin(),
                              moving.end(), std::inserter(stillReported, stillReported.begin()));
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
                if (debug) { std::printf("hold J%d: drift %+.1f, back to %.1f\n", j, drift, held); }
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

        if (moving != active) { std::printf(">>> jogging joints %s\n", fmtJoints(moving).c_str()); }
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
    struct sigaction sa{};
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
            std::printf("\nStopping: %s\n", e.what());  // e.g. joystick unplugged
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
