#include "mycobot.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

// Helper to log current angles
void print_current_angles(const std::string& phase, MyCobot& mc) {
    auto angles = mc.get_angles();
    std::cout << phase << ": [ ";
    for (float a : angles) {
        std::cout << a << " ";
    }
    std::cout << "]" << std::endl;
}

// Executes dip (lowering and raising) at a fixed base angle (j1_angle)
void execute_dip_cycle(MyCobot& mc, float j1_angle) {
    const float max_delta_j2 = 65.0f;   // Maximum safe forward/downward reach
    const float max_delta_j3 = -95.0f;  // Maximum elbow bend
    const int interpolation_steps = 20; // Trajectory resolution
    const int step_delay_ms = 40;       // ~25 Hz update rate

    // 1. Lowering phase
    for (int step = 1; step <= interpolation_steps; ++step) {
        float progress = static_cast<float>(step) / interpolation_steps;
        float d_j2 = progress * max_delta_j2;
        float d_j3 = progress * max_delta_j3;
        float d_j4 = -(d_j2 + d_j3); // Keep end-effector leveled

        std::vector<float> target = {
            j1_angle,
            d_j2,
            d_j3,
            d_j4,
            0.0f,
            0.0f
        };

        mc.send_angles(target, 70);
        std::this_thread::sleep_for(std::chrono::milliseconds(step_delay_ms));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 2. Raising phase back to upright zero configuration
    for (int step = interpolation_steps - 1; step >= 0; --step) {
        float progress = static_cast<float>(step) / interpolation_steps;
        float d_j2 = progress * max_delta_j2;
        float d_j3 = progress * max_delta_j3;
        float d_j4 = -(d_j2 + d_j3);

        std::vector<float> target = {
            j1_angle,
            d_j2,
            d_j3,
            d_j4,
            0.0f,
            0.0f
        };

        mc.send_angles(target, 70);
        std::this_thread::sleep_for(std::chrono::milliseconds(step_delay_ms));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

int main() {
    MyCobot mc;

    std::cout << "Connecting to /dev/ttyTHS1..." << std::endl;
    if (!mc.connect("/dev/ttyTHS1")) {
        std::cerr << "Failed to open serial port!" << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Initialize and engage servo power
    std::cout << "Powering on motors..." << std::endl;
    mc.power_on();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Initial safe zero position
    std::cout << "Moving to default zero position..." << std::endl;
    mc.send_angles({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 25);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Full rotation sweep parameters
    // Joint 1 mechanical span is approximately -165 to +165 deg
    const float start_base_angle = -160.0f;
    const float end_base_angle   =  160.0f;
    const int total_rotation_stops = 8; // Number of dips around the perimeter

    std::cout << "Starting 360-degree sweep cycle with end-effector pitch stabilization..." << std::endl;

    for (int i = 0; i <= total_rotation_stops; ++i) {
        float base_angle = start_base_angle + i * ((end_base_angle - start_base_angle) / total_rotation_stops);

        std::cout << "\n>>> Stop " << (i + 1) << "/" << (total_rotation_stops + 1)
                  << " | Rotating base (J1) to: " << base_angle << " deg" << std::endl;

        // Rotate base while arm is upright/retracted (safe rotation)
        mc.send_angles({base_angle, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 35);
        std::this_thread::sleep_for(std::chrono::milliseconds(1800));

        // Perform maximum dip and return to zero
        std::cout << "Executing vertical dip cycle..." << std::endl;
        execute_dip_cycle(mc, base_angle);
    }

    // Return to default home position
    std::cout << "\nReturning to central home position [0, 0, 0, 0, 0, 0]..." << std::endl;
    mc.send_angles({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 30);
    std::this_thread::sleep_for(std::chrono::seconds(4));

    print_current_angles("Final pose", mc);
    std::cout << "Full sweep cycle completed successfully!" << std::endl;

    return 0;
}
