// pid_tool.cpp - read or change the servo PID values of the myCobot 280,
// using the C++ library (mycobot.hpp).
//
// Start it without arguments for an interactive menu:
//   ./pid_tool
//
//     1) read     show P, I, D of all joints (changes nothing)
//     2) set      change one joint
//     3) all      go through all 6 joints ("joint 1 <p> <i> <d>: ")
//     4) factory  put one joint back to its factory values
//     q) quit
//
// The same commands also work directly as arguments (useful in scripts):
//   ./pid_tool read
//   ./pid_tool set <joint> <P> <I> <D>
//   ./pid_tool factory <joint>
//   ./pid_tool all
//
// Nothing is written without typing YES. Run `read` first: if it prints
// exactly the factory table, the protocol is confirmed and writing is safe.
// Stop every other program using the serial port first (joystick program,
// ROS2 bridge, driver container).

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "mycobot.hpp"

namespace {

// ---- Input helpers ---------------------------------------------------------

// Print `text`, read one line, trim spaces. nullopt at end of input (Ctrl+D).
std::optional<std::string> prompt(const std::string& text) {
    std::printf("%s", text.c_str());
    std::fflush(stdout);
    std::string line;
    if (!std::getline(std::cin, line)) return std::nullopt;
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos) return std::string();
    const auto last = line.find_last_not_of(" \t\r");
    return line.substr(first, last - first + 1);
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

// Ask for a joint number. nullopt if the user cancels (Enter, q, end of input).
std::optional<int> askJoint() {
    while (true) {
        const auto line = prompt("joint (1-6, Enter = cancel): ");
        if (!line || line->empty() || lower(*line) == "q") return std::nullopt;
        std::istringstream in(*line);
        int joint = 0;
        std::string extra;
        if ((in >> joint) && !(in >> extra) && joint >= 1 && joint <= 6) return joint;
        std::printf("  Please type a joint number from 1 to 6\n");
    }
}

enum class Answer { Value, Keep, Quit };

// Ask "joint N <p> <i> <d> (now ...): " until the input is valid.
// Value: `out` holds checked values. Keep: empty line. Quit: q or end of input.
Answer askPid(int joint, const mycobot::Pid& now, mycobot::Pid& out) {
    while (true) {
        const auto line =
            prompt("joint " + std::to_string(joint) + " <p> <i> <d> (now " + std::to_string(now.p) +
                   " " + std::to_string(now.i) + " " + std::to_string(now.d) + "): ");
        if (!line || lower(*line) == "q") return Answer::Quit;
        if (line->empty()) return Answer::Keep;

        std::istringstream in(*line);
        int p = 0, i = 0, d = 0;
        std::string extra;
        if (!(in >> p >> i >> d) || (in >> extra)) {
            std::printf("  Please type exactly three whole numbers, e.g. 25 4 25\n");
            continue;
        }
        try {
            mycobot::MyCobot280::checkPidBounds(joint, {p, i, d});
        } catch (const std::invalid_argument& e) {
            std::printf("  %s - try again\n", e.what());
            continue;
        }
        out = {p, i, d};
        return Answer::Value;
    }
}

bool confirmYes(const std::string& text) {
    const auto answer = prompt(text + " Type YES to continue: ");
    if (answer && *answer == "YES") return true;
    std::printf("Aborted - nothing was changed.\n");
    return false;
}

void printPid(const mycobot::Pid& pid) { std::printf("P %3d  I %3d  D %3d", pid.p, pid.i, pid.d); }

bool same(const mycobot::Pid& a, const mycobot::Pid& b) {
    return a.p == b.p && a.i == b.i && a.d == b.d;
}

// ---- Commands --------------------------------------------------------------

int readAll(mycobot::MyCobot280& mc) {
    std::printf("Joint   current                  factory\n");
    int differences = 0;
    int failures = 0;
    for (int j = 1; j <= 6; ++j) {
        std::printf("J%d      ", j);
        const auto pid = mc.getPid(j);
        const mycobot::Pid& factory = mycobot::FACTORY_PID[j - 1];
        if (pid) {
            printPid(*pid);
        } else {
            std::printf("(no answer)          ");
            ++failures;
        }
        std::printf("      ");
        printPid(factory);
        if (pid && !same(*pid, factory)) {
            std::printf("   <- differs");
            ++differences;
        }
        std::printf("\n");
    }

    if (failures > 0) {
        std::printf(
            "\nSome joints did not answer. Check the GET_SERVO_DATA code in\n"
            "mycobot.hpp against pymycobot before changing anything.\n");
        return 1;
    }
    if (differences > 0) {
        std::printf("\n%d joint(s) differ from the factory table.\n", differences);
    }
    return 0;
}

// Write one joint after showing the change and asking for YES.
int setOne(mycobot::MyCobot280& mc, int joint, const mycobot::Pid& wanted) {
    mycobot::MyCobot280::checkPidBounds(joint, wanted);  // fail early, before asking

    const auto current = mc.getPid(joint);
    if (!current) {
        std::printf("J%d did not answer: nothing was changed.\n", joint);
        return 1;
    }
    if (same(*current, wanted)) {
        std::printf("J%d already has these values - nothing to do.\n", joint);
        return 0;
    }
    std::printf("J%d now: ", joint);
    printPid(*current);
    std::printf("\nJ%d new: ", joint);
    printPid(wanted);
    std::printf("\n");

    if (!confirmYes("This changes the servo's stored settings.")) return 1;
    if (mc.setPid(joint, wanted)) {
        std::printf("Done: J%d verified.\n", joint);
        return 0;
    }
    std::printf("!!! Some values could not be verified. Run `read`.\n");
    return 1;
}

// Menu "set": ask for a joint, then its values.
int setInteractive(mycobot::MyCobot280& mc) {
    const auto joint = askJoint();
    if (!joint) return 1;
    const auto current = mc.getPid(*joint);
    if (!current) {
        std::printf("J%d did not answer: nothing was changed.\n", *joint);
        return 1;
    }
    mycobot::Pid wanted{};
    if (askPid(*joint, *current, wanted) != Answer::Value) {
        std::printf("Nothing was changed.\n");
        return 1;
    }
    return setOne(mc, *joint, wanted);
}

// Menu "factory": ask for a joint, then restore it.
int factoryInteractive(mycobot::MyCobot280& mc) {
    const auto joint = askJoint();
    if (!joint) return 1;
    return setOne(mc, *joint, mycobot::FACTORY_PID[*joint - 1]);
}

// Ask for P I D of each joint in turn, then write all changes after one YES.
int setAll(mycobot::MyCobot280& mc) {
    std::array<mycobot::Pid, 6> current{};
    for (int j = 1; j <= 6; ++j) {
        const auto pid = mc.getPid(j);
        if (!pid) {
            std::printf("J%d did not answer: nothing was changed.\n", j);
            return 1;
        }
        current[j - 1] = *pid;
    }

    std::printf(
        "Type P I D for each joint, separated by spaces (e.g. 25 4 25).\n"
        "Press Enter alone to keep a joint unchanged, or type q to quit.\n\n");

    std::array<mycobot::Pid, 6> wanted = current;
    for (int j = 1; j <= 6; ++j) {
        if (askPid(j, current[j - 1], wanted[j - 1]) == Answer::Quit) {
            std::printf("\nAborted - nothing was changed.\n");
            return 1;
        }
    }

    std::printf("\nJoint   current                  new\n");
    int changes = 0;
    for (int j = 1; j <= 6; ++j) {
        const bool changed = !same(current[j - 1], wanted[j - 1]);
        changes += changed;
        std::printf("J%d      ", j);
        printPid(current[j - 1]);
        std::printf("      ");
        printPid(wanted[j - 1]);
        std::printf(changed ? "   <- change\n" : "\n");
    }
    if (changes == 0) {
        std::printf("\nNothing to change.\n");
        return 0;
    }

    std::printf("\n");
    if (!confirmYes("This changes " + std::to_string(changes) + " servo(s)' stored settings.")) {
        return 1;
    }

    bool allOk = true;
    for (int j = 1; j <= 6; ++j) {
        if (same(current[j - 1], wanted[j - 1])) continue;
        if (mc.setPid(j, wanted[j - 1])) {
            std::printf("  J%d verified\n", j);
        } else {
            std::printf("  !!! J%d could not be verified\n", j);
            allOk = false;
        }
    }
    std::printf(allOk ? "Done.\n" : "Some joints failed. Run `read`.\n");
    return allOk ? 0 : 1;
}

// ---- Menu ------------------------------------------------------------------

int menu(mycobot::MyCobot280& mc) {
    std::printf("myCobot 280 - servo PID tool\n");
    while (true) {
        std::printf(
            "\n"
            "  1) read     show P, I, D of all joints\n"
            "  2) set      change one joint\n"
            "  3) all      go through all 6 joints\n"
            "  4) factory  put one joint back to factory values\n"
            "  q) quit\n");
        const auto choice = prompt("> ");
        if (!choice) break;  // end of input (Ctrl+D)
        const std::string c = lower(*choice);
        std::printf("\n");

        try {
            if (c == "1" || c == "read") {
                readAll(mc);
            } else if (c == "2" || c == "set") {
                setInteractive(mc);
            } else if (c == "3" || c == "all") {
                setAll(mc);
            } else if (c == "4" || c == "factory") {
                factoryInteractive(mc);
            } else if (c == "q" || c == "quit" || c == "exit") {
                break;
            } else if (!c.empty()) {
                std::printf("Unknown command \"%s\" - type 1-4 or q\n", choice->c_str());
            }
        } catch (const std::exception& e) {
            // One failed command (e.g. a value out of bounds) does not end the tool
            std::printf("Error: %s\n", e.what());
        }
    }
    std::printf("Bye.\n");
    return 0;
}

std::string port() { return mycobot::envOr("MYCOBOT_PORT", mycobot::MyCobot280::DEFAULT_PORT); }

void usage() {
    std::printf(
        "Usage:\n"
        "  ./pid_tool                          interactive menu\n"
        "  ./pid_tool read\n"
        "  ./pid_tool set <joint 1-6> <P> <I> <D>\n"
        "  ./pid_tool factory <joint 1-6>\n"
        "  ./pid_tool all\n");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 1) {
            mycobot::MyCobot280 mc(port());
            return menu(mc);
        }

        const std::string action = argv[1];
        if (action == "read" && argc == 2) {
            mycobot::MyCobot280 mc(port());
            return readAll(mc);
        }
        if (action == "set" && argc == 6) {
            const int joint = std::atoi(argv[2]);
            const mycobot::Pid pid{std::atoi(argv[3]), std::atoi(argv[4]), std::atoi(argv[5])};
            mycobot::MyCobot280 mc(port());
            return setOne(mc, joint, pid);
        }
        if (action == "factory" && argc == 3) {
            const int joint = std::atoi(argv[2]);
            if (joint < 1 || joint > 6) {
                std::printf("joint must be 1..6\n");
                return 1;
            }
            mycobot::MyCobot280 mc(port());
            return setOne(mc, joint, mycobot::FACTORY_PID[joint - 1]);
        }
        if (action == "all" && argc == 2) {
            mycobot::MyCobot280 mc(port());
            return setAll(mc);
        }
        usage();
        return 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
}