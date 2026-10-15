// pid_tool.cpp - read or change the servo PID values of the myCobot 280,
// using the C++ library (mycobot.hpp).
//
// Usage:
//   ./pid_tool read                  print P, I, D of all joints and compare
//                                    them with FACTORY_PID (changes nothing)
//   ./pid_tool set <joint> <P> <I> <D>
//                                    change one joint, after typing YES
//   ./pid_tool factory <joint>       put one joint back to its factory values
//
// Run `read` first: if it prints exactly the factory table, the protocol is
// confirmed and `set` is safe to use. Stop every other program using the
// serial port first (joystick program, ROS2 bridge, driver container).

#include "mycobot.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <string>

namespace {

void usage() {
    std::printf("Usage:\n"
                "  ./pid_tool read\n"
                "  ./pid_tool set <joint 1-6> <P> <I> <D>\n"
                "  ./pid_tool factory <joint 1-6>\n");
}

void printPid(const char* label, const mycobot::Pid& pid) {
    std::printf("%s P %3d  I %3d  D %3d", label, pid.p, pid.i, pid.d);
}

int readAll(mycobot::MyCobot280& mc) {
    std::printf("Joint   current                  factory\n");
    int differences = 0;
    int failures = 0;
    for (int j = 1; j <= 6; ++j) {
        std::printf("J%d     ", j);
        const auto pid = mc.getPid(j);
        const mycobot::Pid& factory = mycobot::FACTORY_PID[j - 1];
        if (!pid) {
            std::printf(" (no answer)            ");
            ++failures;
        } else {
            printPid("", *pid);
        }
        std::printf("     ");
        printPid("", factory);
        if (pid && (pid->p != factory.p || pid->i != factory.i || pid->d != factory.d)) {
            std::printf("   <- differs");
            ++differences;
        }
        std::printf("\n");
    }

    std::printf("\n");
    if (failures > 0) {
        std::printf("Some joints did not answer. Check the GET_SERVO_DATA code in\n"
                    "mycobot.hpp against pymycobot before using `set`.\n");
        return 1;
    }
    if (differences > 0) {
        std::printf("%d joint(s) differ from the factory table. If you never changed\n"
                    "them, the reply format may be decoded wrongly: do not use `set` yet.\n",
                    differences);
        return 0;
    }
    std::printf("All joints match the factory table: reading works.\n");
    return 0;
}

int setOne(mycobot::MyCobot280& mc, int joint, const mycobot::Pid& wanted) {
    mycobot::MyCobot280::checkPidBounds(joint, wanted);   // fail early, before asking

    const auto current = mc.getPid(joint);
    if (!current) {
        std::printf("J%d did not answer: nothing was changed.\n", joint);
        return 1;
    }
    printPid(("J" + std::to_string(joint) + " now:").c_str(), *current);
    std::printf("\n");
    printPid(("J" + std::to_string(joint) + " new:").c_str(), wanted);
    std::printf("\n\nThis changes the servo's stored settings. Type YES to continue: ");

    std::string answer;
    std::getline(std::cin, answer);
    if (answer != "YES") {
        std::printf("Aborted - nothing was changed.\n");
        return 1;
    }

    if (mc.setPid(joint, wanted)) {
        std::printf("Done: J%d verified.\n", joint);
        return 0;
    }
    std::printf("!!! Some values could not be verified. Run `./pid_tool read`.\n");
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 1;
    }
    const std::string action = argv[1];

    try {
        if (action == "read" && argc == 2) {
            mycobot::MyCobot280 mc;
            return readAll(mc);
        }
        if (action == "set" && argc == 6) {
            const int joint = std::atoi(argv[2]);
            const mycobot::Pid pid{std::atoi(argv[3]), std::atoi(argv[4]), std::atoi(argv[5])};
            mycobot::MyCobot280 mc;
            return setOne(mc, joint, pid);
        }
        if (action == "factory" && argc == 3) {
            const int joint = std::atoi(argv[2]);
            if (joint < 1 || joint > 6) {
                std::printf("joint must be 1..6\n");
                return 1;
            }
            mycobot::MyCobot280 mc;
            return setOne(mc, joint, mycobot::FACTORY_PID[joint - 1]);
        }
        usage();
        return 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
}