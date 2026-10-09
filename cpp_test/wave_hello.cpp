#include "mycobot.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

// Function to execute the waving motion
void execute_wave(MyCobot& mc, int wave_count) {
    // 1. Move to a greeting posture
    // J1=0 (Center)
    // J2=30 (Lean forward/down slightly)
    // J3=-90 (Bend elbow up sharply to simulate a raised arm)
    // J4=-30 (Adjust wrist pitch to point "hand" up)
    // J5=0, J6=0 (Neutral wrist)
    std::vector<float> greeting_pose = {0.0f, 30.0f, -90.0f, -30.0f, 0.0f, 0.0f};
    
    std::cout << "Raising arm to greeting posture..." << std::endl;
    mc.send_angles(greeting_pose, 35);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "Starting to wave!" << std::endl;
    
    // 2. Loop to move Joint 5 (wrist yaw) left and right
    for (int i = 0; i < wave_count; ++i) {
        std::cout << "Wave cycle " << (i + 1) << "..." << std::endl;
        
        // Swing right (+60 degrees) at high speed (70)
        mc.send_angle(5, 60.0f, 70);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Swing left (-60 degrees) at high speed (70)
        mc.send_angle(5, -60.0f, 70);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 3. Re-center the wrist after waving
    std::cout << "Re-centering wrist..." << std::endl;
    mc.send_angle(5, 0.0f, 50);
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main() {
    MyCobot mc;

    std::cout << "Connecting to /dev/ttyTHS1..." << std::endl;
    if (!mc.connect("/dev/ttyTHS1")) {
        std::cerr << "Failed to open serial port!" << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Power on the motors
    std::cout << "Powering on motors..." << std::endl;
    mc.power_on();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Ensure it starts from a safe zero position
    std::cout << "Moving to safe zero position..." << std::endl;
    mc.send_angles({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 30);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Execute the waving sequence (e.g., wave 5 times)
    execute_wave(mc, 5);

    // Return back to safe zero position
    std::cout << "Returning to zero position before exit..." << std::endl;
    mc.send_angles({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 30);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "Waving sequence finished successfully!" << std::endl;
    return 0;
}
