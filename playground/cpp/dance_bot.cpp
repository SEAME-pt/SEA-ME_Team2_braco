#include "mycobot.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

// Dance Move 1: The Twist (Swinging base and wrist)
void dance_the_twist(MyCobot& mc) {
    std::cout << "--- Move 1: The Twist ---" << std::endl;
    
    // Get into a bent arm posture
    // J1=0, J2=30, J3=-50, J4=-20, J5=0, J6=0
    mc.send_angles({0.0f, 30.0f, -50.0f, -20.0f, 0.0f, 0.0f}, 40);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Swing left and right 3 times
    for (int i = 0; i < 3; ++i) {
        // Swing left, rotate wrist left
        mc.send_angles({45.0f, 30.0f, -50.0f, -20.0f, -60.0f, 0.0f}, 65);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        
        // Swing right, rotate wrist right
        mc.send_angles({-45.0f, 30.0f, -50.0f, -20.0f, 60.0f, 0.0f}, 65);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
    }
    
    // Re-center
    mc.send_angles({0.0f, 30.0f, -50.0f, -20.0f, 0.0f, 0.0f}, 50);
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

// Dance Move 2: The Headbang (Rhythmic up and down)
void dance_the_headbang(MyCobot& mc) {
    std::cout << "--- Move 2: The Headbang ---" << std::endl;
    
    // Posture looking like a head pointing forward
    mc.send_angles({0.0f, 10.0f, -60.0f, 0.0f, 90.0f, 0.0f}, 40);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Nod up and down aggressively 4 times
    for (int i = 0; i < 4; ++i) {
        // Nod down
        mc.send_angle(2, 40.0f, 80);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        
        // Nod up
        mc.send_angle(2, -10.0f, 80);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
}

// Dance Move 3: The Wave (Undulating arm motion)
void dance_the_wave(MyCobot& mc) {
    std::cout << "--- Move 3: The Wave ---" << std::endl;
    
    // Wave cycle 2 times
    for (int i = 0; i < 2; ++i) {
        // Curl inwards (compress)
        mc.send_angles({0.0f, -20.0f, -90.0f, 45.0f, 0.0f, 45.0f}, 50);
        std::this_thread::sleep_for(std::chrono::milliseconds(900));
        
        // Stretch outwards (extend)
        mc.send_angles({0.0f, 45.0f, 0.0f, -45.0f, 0.0f, -45.0f}, 50);
        std::this_thread::sleep_for(std::chrono::milliseconds(900));
    }
}

int main() {
    MyCobot mc;

    std::cout << "Connecting to /dev/ttyTHS1..." << std::endl;
    if (!mc.connect("/dev/ttyTHS1")) {
        std::cerr << "Failed to open serial port!" << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Powering on motors..." << std::endl;
    mc.power_on();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 1. Start at safe zero position
    std::cout << "Moving to stage (zero position)..." << std::endl;
    mc.send_angles({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 30);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // 2. Perform the dance routine
    std::cout << ">>> LET'S DANCE! <<<" << std::endl;
    
    dance_the_twist(mc);
    dance_the_headbang(mc);
    dance_the_wave(mc);
    
    // 3. Take a bow (curvar-se para agradecer)
    std::cout << "--- Taking a bow ---" << std::endl;
    mc.send_angles({0.0f, 60.0f, -90.0f, -60.0f, 0.0f, 0.0f}, 30);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // 4. Return to zero
    std::cout << "Dance finished. Returning to zero position..." << std::endl;
    mc.send_angles({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 30);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "Program ended successfully!" << std::endl;
    return 0;
}
