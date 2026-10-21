// display.cpp - implementation of ui::TerminalDisplay (see display.hpp).

#include "display.hpp"

#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>

namespace ui
{

namespace
{

// ANSI escape codes
const char * const ALT_SCREEN_ON = "\x1b[?1049h";
const char * const ALT_SCREEN_OFF = "\x1b[?1049l";
const char * const CURSOR_HIDE = "\x1b[?25l";
const char * const CURSOR_SHOW = "\x1b[?25h";
const char * const CLEAR_SCREEN = "\x1b[2J";
const char * const CURSOR_HOME = "\x1b[H";
const char * const CLEAR_LINE_END = "\x1b[K";
const char * const CLEAR_BELOW = "\x1b[J";
const char * const RESET = "\x1b[0m";
const char * const BOLD = "\x1b[1m";
const char * const DIM = "\x1b[2m";
const char * const RED = "\x1b[31m";
const char * const YELLOW = "\x1b[33m";
const char * const CYAN = "\x1b[36m";

constexpr int BAR_WIDTH = 25;

// Coloured block showing the speed mode colour (24-bit colour terminals).
std::string swatch(uint8_t r, uint8_t g, uint8_t b)
{
  char buf[48];
  std::snprintf(buf, sizeof(buf), "\x1b[48;2;%d;%d;%dm    ", r, g, b);
  return std::string(buf) + RESET;
}

// |-----+-----o-------| : '+' = 0 deg (home), 'o' = current angle
std::string positionBar(double angle, double min, double max)
{
  std::string bar(BAR_WIDTH, '-');
  auto index = [&](double value) {
    const double ratio = (value - min) / (max - min);
    return std::clamp(static_cast<int>(std::lround(ratio * (BAR_WIDTH - 1))), 0, BAR_WIDTH - 1);
  };
  if (min < 0.0 && max > 0.0) bar[index(0.0)] = '+';
  bar[index(angle)] = 'o';
  return "|" + bar + "|";
}

}  // namespace

TerminalDisplay::TerminalDisplay(bool enabled, std::chrono::milliseconds refresh)
: active_(enabled && isatty(STDOUT_FILENO)), refresh_(refresh)
{
  if (active_) {
    std::fflush(stdout);
    writeAll(std::string(ALT_SCREEN_ON) + CURSOR_HIDE + CLEAR_SCREEN);
  }
}

TerminalDisplay::~TerminalDisplay()
{
  if (active_) {
    writeAll(std::string(RESET) + CURSOR_SHOW + ALT_SCREEN_OFF);
  }
}

void TerminalDisplay::addMessage(const std::string & text) noexcept
{
  try {
    if (!active_) {
      std::printf("%s\n", text.c_str());
      return;
    }
    if (!messages_.empty() && messages_.back().first == text) {
      ++messages_.back().second;  // same as last message: count it
      return;
    }
    messages_.emplace_back(text, 1);
    while (messages_.size() > MAX_MESSAGES) messages_.pop_front();
  } catch (...) {
    // Never let the display affect the arm program
  }
}

void TerminalDisplay::show(const Status & status, bool force) noexcept
{
  if (!active_) return;
  try {
    const auto now = std::chrono::steady_clock::now();
    if (!force && now - lastDraw_ < refresh_) return;
    lastDraw_ = now;
    render(status);
  } catch (...) {
    // Never let the display affect the arm program
  }
}

void TerminalDisplay::render(const Status & s)
{
  std::string out = CURSOR_HOME;
  char buf[256];
  auto line = [&](const std::string & text = "") {
    out += text;
    out += CLEAR_LINE_END;
    out += "\n";
  };

  line(std::string(BOLD) + " myCobot 280 - joystick control" + RESET);
  line();

  // Speed mode
  std::snprintf(
    buf, sizeof(buf), "  %-7s  cap %3d %%   max coast %2.0f deg", s.modeName.c_str(), s.speedCap,
    s.maxCoast);
  line(" Speed  " + swatch(s.r, s.g, s.b) + buf);

  // Arm state
  std::string state;
  if (!s.feedbackOk) {
    state = std::string(RED) + BOLD + "NO ANGLE FEEDBACK" + RESET;
  } else if (s.homing) {
    state = std::string(CYAN) + BOLD + "HOMING to 0" + RESET;
  } else {
    std::string joints;
    for (int j = 0; j < 6; ++j) {
      if (s.moving[j]) joints += " J" + std::to_string(j + 1);
    }
    state = joints.empty() ? std::string(DIM) + "idle" + RESET
                           : std::string(BOLD) + "JOGGING" + RESET + joints;
  }
  line(" State  " + state);
  line();

  // Joint table
  line(std::string(DIM) + " Joint   Angle (deg)   Soft limits     Position (+ = 0 deg)" + RESET);
  for (int j = 0; j < 6; ++j) {
    const auto [min, max] = s.limits[j];
    std::string row;
    std::snprintf(buf, sizeof(buf), " J%d   ", j + 1);
    row += buf;
    if (s.angles) {
      const double angle = (*s.angles)[j];
      std::snprintf(buf, sizeof(buf), "%9.2f     %5.0f .. %-5.0f  ", angle, min, max);
      row += buf;
      row += positionBar(angle, min, max);
    } else {
      std::snprintf(buf, sizeof(buf), "%9s     %5.0f .. %-5.0f  ", "--", min, max);
      row += buf;
    }
    if (s.atLimit[j]) {
      row += std::string("  ") + YELLOW + BOLD + "LIMIT" + RESET;
    } else if (s.moving[j]) {
      row += std::string("  ") + BOLD + "<" + RESET;
    }
    line(row);
  }
  line();

  // Message log (fixed height so the layout does not jump)
  line(std::string(DIM) + " Messages" + RESET);
  for (size_t i = 0; i < MAX_MESSAGES; ++i) {
    if (i < messages_.size()) {
      std::string text = "  " + messages_[i].first;
      if (messages_[i].second > 1) text += "  (x" + std::to_string(messages_[i].second) + ")";
      line(text);
    } else {
      line();
    }
  }
  line();
  line(
    std::string(DIM) +
    " Sticks/D-pad/L1/R1 jog | A/B gripper | X home | Button 9 speed | Ctrl+C quit" + RESET);

  out += CLEAR_BELOW;
  writeAll(out);
}

// Write the whole frame with as few system calls as possible (no flicker).
void TerminalDisplay::writeAll(const std::string & text)
{
  size_t done = 0;
  while (done < text.size()) {
    ssize_t n = write(STDOUT_FILENO, text.data() + done, text.size() - done);
    if (n < 0) {
      if (errno == EINTR) continue;
      return;  // terminal gone: give up silently
    }
    done += static_cast<size_t>(n);
  }
}

}  // namespace ui
