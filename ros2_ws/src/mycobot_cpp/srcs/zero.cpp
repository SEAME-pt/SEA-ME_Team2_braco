// zero.cpp - move all 6 joints of the myCobot 280 to 0 deg.
// C++ version of zero.py, using the MyCobot280 class from mycobot.hpp.
//
// Build:
//     g++ -O2 -std=c++17 -o zero zero.cpp mycobot.cpp

#include <chrono>
#include <cstdio>
#include <exception>
#include <optional>
#include <thread>

#include "mycobot.hpp"

using namespace std::chrono_literals;

namespace {

void printAngles(const char* label, const std::optional<mycobot::Angles>& angles) {
    if (!angles) {
        std::printf("%s (no valid reading)\n", label);
        return;
    }
    std::printf("%s [", label);
    for (size_t i = 0; i < angles->size(); ++i) {
        std::printf("%s%.2f", i ? ", " : "", (*angles)[i]);
    }
    std::printf("]\n");
}

}  // namespace

int main() {
    try {
        mycobot::MyCobot280 mc(mycobot::envOr("MYCOBOT_PORT", mycobot::MyCobot280::DEFAULT_PORT));
        std::this_thread::sleep_for(1s);

        printAngles("Before:", mc.getAngles());

        // Send the command for all 6 joints to move simultaneously
        mc.sendAngles(mycobot::ZERO_ANGLES, mycobot::MOVE_SPEED);

        // Wait for the movement to finish before reading the angles again
        std::this_thread::sleep_for(3s);

        printAngles("After:", mc.getAngles());
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;  // systemd sees the failure, same as the Python version
    }
}
int x = 1;
