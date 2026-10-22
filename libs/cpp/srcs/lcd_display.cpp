// lcd_display.cpp - implementation of ui::LcdDisplay (see lcd_display.hpp).

#include "lcd_display.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

namespace ui
{

namespace
{

constexpr uint8_t LCD_ADDR = 0x3E;
constexpr uint8_t RGB_ADDR = 0x60;

// LCD controller: first byte selects command or character data
constexpr uint8_t LCD_CMD = 0x80;
constexpr uint8_t LCD_DATA = 0x40;
constexpr uint8_t LCD_ROW0 = 0x80;  // "set cursor" command, row 0, column 0
constexpr uint8_t LCD_ROW1 = 0xC0;  // "set cursor" command, row 1, column 0

// Backlight driver registers. Colour order as in Waveshare's RGB1602
// library; if colours come out swapped, swap these three values.
constexpr uint8_t REG_MODE1 = 0x00;
constexpr uint8_t REG_MODE2 = 0x01;
constexpr uint8_t REG_OUTPUT = 0x08;
constexpr uint8_t REG_BLUE = 0x02;
constexpr uint8_t REG_GREEN = 0x03;
constexpr uint8_t REG_RED = 0x04;

// Placeholder that never matches a printable character: forces a full rewrite.
const std::string UNKNOWN_LINE(LcdDisplay::COLS, '\0');

void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// Exactly 16 printable ASCII characters (the LCD has no other glyphs).
std::string fit(const std::string & text)
{
  std::string out = text.substr(0, LcdDisplay::COLS);
  out.resize(LcdDisplay::COLS, ' ');
  for (char & c : out) {
    if (c < 0x20 || c > 0x7E) c = '?';
  }
  return out;
}

std::string center(const std::string & text)
{
  if (text.size() >= static_cast<size_t>(LcdDisplay::COLS)) return text;
  return std::string((LcdDisplay::COLS - text.size()) / 2, ' ') + text;
}

// Three angles in fixed 5-character columns, e.g. " -150  -12  128".
// Always 15 characters for -168..168, so no value is ever cut off.
std::string angleRow(const std::array<double, 6> & a, int first)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%5ld%5ld%5ld", std::lround(a[first]), std::lround(a[first + 1]),
                std::lround(a[first + 2]));
  return buf;
}

// Three labelled temperatures, e.g. "1:31  2:33  3:35" (exactly 16 characters).
std::string tempRow(const std::array<int, 6> & t, int first)
{
  auto c = [](int v) { return v < 0 ? 0 : (v > 99 ? 99 : v); };  // keep 2 digits
  char buf[80];  // large enough for any int, so no truncation warnings
  std::snprintf(buf, sizeof(buf), "%d:%2d  %d:%2d  %d:%2d", first + 1, c(t[first]), first + 2,
                c(t[first + 1]), first + 3, c(t[first + 2]));
  return buf;
}

}  // namespace

// ---------------------------------------------------------------- lifecycle

LcdDisplay::LcdDisplay(const std::string & bus, std::chrono::milliseconds refresh)
: refresh_(refresh), shown_{UNKNOWN_LINE, UNKNOWN_LINE}
{
  fd_ = open(bus.c_str(), O_RDWR);
  if (fd_ < 0) {
    error_ = bus + ": " + std::strerror(errno);
    return;
  }
  if (!initBacklight() || !initLcd()) {
    error_ = "no answer from the LCD on " + bus + " (check wiring and power)";
    close(fd_);
    fd_ = -1;
    return;
  }
  ok_ = true;
}

LcdDisplay::~LcdDisplay()
{
  if (fd_ >= 0) close(fd_);
}

// Same sequence as the Python driver that lit the backlight on this module.
bool LcdDisplay::initBacklight()
{
  bool good = writeReg(RGB_ADDR, REG_MODE1, 0x00) && writeReg(RGB_ADDR, REG_MODE2, 0x00) &&
              writeReg(RGB_ADDR, REG_OUTPUT, 0xAA);
  for (uint8_t reg = 0x02; good && reg <= 0x05; ++reg) {
    good = writeReg(RGB_ADDR, reg, 0xFF);  // all channels fully on
  }
  if (good) backlight_ = {255, 255, 255};
  return good;
}

// Same command sequence and delays as the Python driver.
bool LcdDisplay::initLcd()
{
  sleepMs(50);
  if (!command(0x38)) return false;  // function set: 8-bit, 2 lines
  sleepMs(5);
  if (!command(0x39)) return false;  // extended instruction set
  sleepMs(5);
  if (!command(0x14) || !command(0x74) || !command(0x54) || !command(0x6F)) return false;
  sleepMs(200);
  if (!command(0x38) || !command(0x0C) || !command(0x01))
    return false;  // normal set, display on, clear
  sleepMs(10);
  return command(0x06);  // entry mode: cursor moves right
}

// ------------------------------------------------------------------ drawing

std::array<std::string, 2> LcdDisplay::linesFor(const Status & s)
{
  std::string joints;
  for (int j = 0; j < 6; ++j) {
    if (s.moving[j]) joints += (joints.empty() ? "J" : ",J") + std::to_string(j + 1);
  }

  if (!s.feedbackOk) return {fit("NO FEEDBACK"), fit("check arm power")};
  if (s.hotJoint >= 1 && s.hotJoint <= 6 && s.temps) {
    const int t = (*s.temps)[s.hotJoint - 1];
    return {fit("SERVO HOT! J" + std::to_string(s.hotJoint)),
            fit(std::to_string(t) + " C - rest arm")};
  }
  if (s.homing) return {fit("ARM MOVING..."), fit(center("HOMING ALL"))};
  if (!joints.empty()) return {fit("ARM MOVING..."), fit(center(joints))};
  if (s.temps) return {fit(tempRow(*s.temps, 0)), fit(tempRow(*s.temps, 3))};
  if (!s.angles) return {fit("ANGLES UNKNOWN"), fit("")};
  return {fit(angleRow(*s.angles, 0)), fit(angleRow(*s.angles, 3))};
}

void LcdDisplay::show(const Status & status, bool force) noexcept
{
  if (!ok_) return;
  try {
    const auto now = std::chrono::steady_clock::now();
    if (!force && now - lastDraw_ < refresh_) return;
    lastDraw_ = now;

    const auto lines = linesFor(status);
    // Stop at the first failure: a disconnected display then costs one
    // failed transfer per refresh instead of dozens.
    if (!updateLine(0, lines[0])) return;
    if (!updateLine(1, lines[1])) return;
    setBacklight(status.r, status.g, status.b);
  } catch (...) {
    // Never let the display affect the arm program
  }
}

bool LcdDisplay::updateLine(int row, const std::string & text)
{
  const std::string want = fit(text);
  std::string & have = shown_[row];
  int cursor = -1;  // column the LCD cursor is at, -1 = unknown

  for (int col = 0; col < COLS; ++col) {
    if (have[col] == want[col]) continue;
    if (cursor != col && !command(static_cast<uint8_t>((row == 0 ? LCD_ROW0 : LCD_ROW1) | col))) {
      have = UNKNOWN_LINE;  // unsure what is shown now: rewrite next time
      return false;
    }
    if (!data(static_cast<uint8_t>(want[col]))) {
      have = UNKNOWN_LINE;
      return false;
    }
    have[col] = want[col];
    cursor = col + 1;  // the LCD advances the cursor by itself
  }
  return true;
}

bool LcdDisplay::setBacklight(uint8_t r, uint8_t g, uint8_t b)
{
  if (backlight_[0] == r && backlight_[1] == g && backlight_[2] == b) return true;
  if (writeReg(RGB_ADDR, REG_RED, r) && writeReg(RGB_ADDR, REG_GREEN, g) &&
      writeReg(RGB_ADDR, REG_BLUE, b)) {
    backlight_ = {r, g, b};
    return true;
  }
  backlight_ = {-1, -1, -1};  // unknown: retry next time
  return false;
}

// ---------------------------------------------------------------- I2C layer

bool LcdDisplay::writeReg(uint8_t addr, uint8_t reg, uint8_t value)
{
  if (fd_ < 0) return false;
  if (currentAddr_ != addr) {
    if (ioctl(fd_, I2C_SLAVE, addr) < 0) return false;
    currentAddr_ = addr;
  }
  const uint8_t buf[2] = {reg, value};
  return write(fd_, buf, sizeof(buf)) == static_cast<ssize_t>(sizeof(buf));
}

bool LcdDisplay::command(uint8_t cmd) { return writeReg(LCD_ADDR, LCD_CMD, cmd); }

bool LcdDisplay::data(uint8_t ch) { return writeReg(LCD_ADDR, LCD_DATA, ch); }

}  // namespace ui
