// lcd_display.hpp - Waveshare LCD1602 RGB (16x2 characters, I2C) status display.
//
// Two chips on the I2C bus:
//   0x3E  LCD controller (commands and characters)
//   0x60  RGB backlight driver (one PWM channel per colour)
//
// show() takes the same ui::Status as the terminal display and decides what
// fits on 2 x 16 characters:
//   idle     servo temperatures "1:31  2:33  3:35" / "4:41  5:38  6:30"
//            (angles instead, if the temperatures cannot be read)
//   too hot  "SERVO HOT! J4" and its temperature (overrides moving/idle)
//   jogging  "ARM MOVING..." and the moving joints
//   homing   "ARM MOVING..." and "HOMING ALL"
// The backlight shows the speed mode colour (same as the Atom LED).
//
// Safety rules (same as the terminal display):
//   * Nothing throws. If the LCD is missing or a write fails, the arm program
//     carries on; the display just shows nothing or retries later.
//   * Only characters that changed are written (each one is an I2C transfer),
//     and redraws are rate-limited, so the control loop is not slowed down.
// The implementation lives in lcd_display.cpp.

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

#include "display.hpp"

namespace ui
{

class LcdDisplay
{
public:
  static constexpr const char * DEFAULT_BUS = "/dev/i2c-0";
  static constexpr int COLS = 16;

  // Opens the bus and initialises the LCD. Never throws: check ok().
  explicit LcdDisplay(const std::string & bus = DEFAULT_BUS,
                      std::chrono::milliseconds refresh = std::chrono::milliseconds(200));
  ~LcdDisplay();  // leaves the last content on the screen

  LcdDisplay(const LcdDisplay &) = delete;
  LcdDisplay & operator=(const LcdDisplay &) = delete;

  bool ok() const { return ok_; }                       // LCD found and initialised
  const std::string & error() const { return error_; }  // why not, if !ok()

  // Redraw from the status, at most once per refresh period unless force.
  void show(const Status & status, bool force = false) noexcept;

  // The two 16-character lines show() would draw for this status.
  // Public so the layout can be tested without hardware.
  static std::array<std::string, 2> linesFor(const Status & status);

private:
  int fd_ = -1;
  bool ok_ = false;
  std::string error_;
  std::chrono::milliseconds refresh_;
  std::chrono::steady_clock::time_point lastDraw_{};
  int currentAddr_ = -1;                         // I2C address selected on fd_
  std::array<std::string, 2> shown_;             // what is on the screen now
  std::array<int, 3> backlight_ = {-1, -1, -1};  // current R, G, B (-1 = unknown)

  bool initBacklight();
  bool initLcd();
  bool updateLine(int row, const std::string & text);  // writes changed chars only
  bool setBacklight(uint8_t r, uint8_t g, uint8_t b);  // skips if unchanged

  bool writeReg(uint8_t addr, uint8_t reg, uint8_t value);
  bool command(uint8_t cmd);
  bool data(uint8_t ch);
};

}  // namespace ui
