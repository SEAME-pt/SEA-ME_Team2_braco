#include "mycobot.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>

int main(int argc, char* argv[]) {
    std::string port = "/dev/ttyTHS1";
    if (argc > 1) {
        port = argv[1];
    }

    std::cout << "========================================" << std::endl;
    std::cout << "   myCobot 280 C++ Hello World Demo     " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Target serial port: " << port << " (Baud: 1000000)" << std::endl;

    MyCobot mc;
    if (!mc.connect(port)) {
        std::cerr << "\n[ERROR] Failed to open serial port: " << port << std::endl;
        std::cerr << "Troubleshooting hints:" << std::endl;
        std::cerr << "  1. Check device existence: ls -l " << port << std::endl;
        std::cerr << "  2. Ensure dialout group permission: sudo usermod -aG dialout $USER (then relog)" << std::endl;
        std::cerr << "  3. If using USB adapter, specify port: ./hello_world /dev/ttyUSB0" << std::endl;
        return 1;
    }

    std::cout << "[OK] Serial connection established." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "[INFO] Powering on servos..." << std::endl;
    mc.power_on();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "[INFO] Reading current joint angles..." << std::endl;
    std::vector<float> angles = mc.get_angles();

    if (angles.size() != 6) {
        std::cerr << "[ERROR] No valid response to get_angles query (check power switch and baud rate)." << std::endl;
        std::cerr << "\nHello-world FAILED: no telemetry received from the robot." << std::endl;
        return 2;
    }

    std::cout << "[OK] Received joint angles: [ ";
    for (size_t i = 0; i < angles.size(); ++i) {
        std::cout << "J" << (i + 1) << ": " << angles[i] << "° ";
    }
    std::cout << "]" << std::endl;

    std::cout << "\n========================================" << std::endl;
    std::cout << "Hello, myCobot! C++ communication verified successfully." << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
