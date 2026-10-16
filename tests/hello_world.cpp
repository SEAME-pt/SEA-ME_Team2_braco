#include "mycobot.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <cstdlib>
#include <termios.h>

// termios takes speed constants (B115200), not plain integers.
static bool baud_to_speed(long baud, speed_t& out) {
    switch (baud) {
        case 9600:    out = B9600;    return true;
        case 19200:   out = B19200;   return true;
        case 38400:   out = B38400;   return true;
        case 57600:   out = B57600;   return true;
        case 115200:  out = B115200;  return true;
        case 230400:  out = B230400;  return true;
        case 460800:  out = B460800;  return true;
        case 921600:  out = B921600;  return true;
        case 1000000: out = B1000000; return true;
        default: return false;
    }
}

int main(int argc, char* argv[]) {
    std::string port = "/dev/ttyTHS1";
    long baud = 1000000;

    // Usage: hello_world [PORT [BAUD]]  or  hello_world [--port PORT] [--baud BAUD]
    int positional = 0;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "--port" || arg == "--baud") && i + 1 < argc) {
            std::string value = argv[++i];
            if (arg == "--port") port = value; else baud = std::atol(value.c_str());
        } else if (positional == 0) {
            port = arg; ++positional;
        } else if (positional == 1) {
            baud = std::atol(arg.c_str()); ++positional;
        } else {
            std::cerr << "Usage: " << argv[0] << " [PORT [BAUD]] | [--port PORT] [--baud BAUD]" << std::endl;
            return 1;
        }
    }

    speed_t speed;
    if (!baud_to_speed(baud, speed)) {
        std::cerr << "[ERROR] Unsupported baud rate: " << baud
                  << " (use 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600 or 1000000)" << std::endl;
        return 1;
    }

    std::cout << "========================================" << std::endl;
    std::cout << "   myCobot 280 C++ Hello World Demo     " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Target serial port: " << port << " (Baud: " << baud << ")" << std::endl;

    MyCobot mc;
    if (!mc.connect(port, speed)) {
        std::cerr << "\n[ERROR] Failed to open serial port: " << port << std::endl;
        std::cerr << "Troubleshooting hints:" << std::endl;
        std::cerr << "  1. Check device existence: ls -l " << port << std::endl;
        std::cerr << "  2. Ensure dialout group permission: sudo usermod -aG dialout $USER (then relog)" << std::endl;
        std::cerr << "  3. If using USB adapter, specify port and baud: ./hello_world /dev/ttyUSB0 115200" << std::endl;
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

    std::cout << "[INFO] Showing LED colors: red, green, blue, then green..." << std::endl;
    const std::chrono::seconds step(1);
    mc.set_color(255, 0, 0);
    std::this_thread::sleep_for(step);
    mc.set_color(0, 255, 0);
    std::this_thread::sleep_for(step);
    mc.set_color(0, 0, 255);
    std::this_thread::sleep_for(step);
    mc.set_color(0, 255, 0);

    std::cout << "\n========================================" << std::endl;
    std::cout << "Hello, myCobot! C++ communication verified successfully." << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
