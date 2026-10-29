// temps_lcd.cpp - show the myCobot 280 servo temperatures on the LCD1602.
//
// Reference example for mycobot.hpp + lcd_display.hpp. It only READS from
// the arm: no motion, no power changes, no settings written.
//
//   ./temps_lcd
//
// Every TEMP_PERIOD it reads the six servo temperatures, prints them in the
// terminal and shows them on the LCD:
//
//   |1:31  2:33  3:35|      normal
//   |4:41  5:38  6:30|
//
//   |SERVO HOT! J4   |      a servo at TEMP_HOT_C or above
//   |66 C - rest arm |
//
// The backlight shows the level: white = normal, yellow = warm, red = hot.
// Ctrl+C quits; the last values stay on the LCD.
//
// Stop other programs using the serial port first (joystick program, driver).

#include <signal.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <thread>

#include "lcd_display.hpp"
#include "mycobot.hpp"

namespace {

constexpr auto TEMP_PERIOD = std::chrono::seconds(2);  // temperatures change slowly

volatile std::sig_atomic_t g_quit = 0;

void onSignal(int) { g_quit = 1; }

// Backlight colour for each temperature level
void setColour(ui::Status& status, mycobot::TempLevel level) {
    const mycobot::Rgb colour = mycobot::tempColour(level);
    status.r = colour.r;
    status.g = colour.g;
    status.b = colour.b;
}

// const char* levelName(mycobot::TempLevel level) {
//     switch (level) {
//         case mycobot::TempLevel::Warm: return "  <- warm";
//         case mycobot::TempLevel::Hot:  return "  <- HOT, let the arm rest";
//         default:                       return "";
//     }
// }

}  // namespace

int main() {
    // Ctrl+C or kill: finish the current step, then exit cleanly
    struct sigaction sa{};
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    try {
        // 1. LCD: never throws, check ok() instead
        ui::LcdDisplay lcd(mycobot::envOr("MYCOBOT_LCD_BUS", ui::LcdDisplay::DEFAULT_BUS));
        if (!lcd.ok()) {
            std::printf("LCD not available (%s) - showing temperatures here only\n",
                        lcd.error().c_str());
        }

        // 2. Arm: throws if the serial port cannot be opened
        mycobot::MyCobot280 mc(mycobot::envOr("MYCOBOT_PORT", mycobot::MyCobot280::DEFAULT_PORT));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::printf("Reading servo temperatures every %lld s. Ctrl+C quits.\n\n",
                    static_cast<long long>(TEMP_PERIOD.count()));

        ui::Status status;  // everything the LCD shows
        setColour(status, mycobot::TempLevel::Normal);

        while (!g_quit) {
            // 3. Read and check the temperatures
            const auto check = mc.checkTemps();

            if (!check) {
                // No answer: the LCD shows "NO FEEDBACK / check arm power"
                status.feedbackOk = false;
                status.temps.reset();
                status.hotJoint = 0;
                std::printf("No answer from the arm\n");
            } else {
                // 4. Fill the status: temperatures, hot warning, colour
                status.feedbackOk = true;
                status.temps = check->temps;
                status.hotJoint =
                    (check->level == mycobot::TempLevel::Hot) ? check->hottestJoint : 0;
                setColour(status, check->level);

                // for (int j = 0; j < 6; ++j) {
                //     std::printf("J%d %3d C   ", j + 1, check->temps[j]);
                // }
                // std::printf("%s\n", levelName(check->level));
            }

            // 5. Draw now (force: we only update every TEMP_PERIOD anyway)
            lcd.show(status, true);

            // Sleep in small steps so Ctrl+C reacts quickly
            const auto wakeUp = std::chrono::steady_clock::now() + TEMP_PERIOD;
            while (!g_quit && std::chrono::steady_clock::now() < wakeUp) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        std::printf("\nStopped. The last values stay on the LCD.\n");
        return 0;

    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
}
