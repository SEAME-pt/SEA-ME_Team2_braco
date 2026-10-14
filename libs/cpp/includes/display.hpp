// display.hpp - full-screen status display in the terminal (SSH or local).
//
// Draws a fixed screen that is updated in place: speed mode, arm state,
// joint angles with their position inside the soft limits, and recent
// messages. Uses plain ANSI escape codes, so it needs no extra libraries.
// The implementation lives in display.cpp.
//
// Safety rules built into the class:
//   * show() is rate-limited, so drawing never slows down the control loop.
//   * No method throws: a display problem can never stop the arm program.
//   * If stdout is not a terminal (e.g. running under systemd) or the display
//     is disabled, nothing is drawn and messages are printed as plain lines.

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>

namespace ui {

// Everything the display shows. The program fills it; the display draws it.
struct Status {
    std::optional<std::array<double, 6>> angles;   // J1..J6, nullopt = unknown
    std::array<std::pair<double, double>, 6> limits{};   // soft limits (min, max)

    std::string modeName;     // speed mode, e.g. "medium"
    int speedCap = 0;         // percent
    double maxCoast = 0.0;    // degrees
    uint8_t r = 0, g = 0, b = 0;   // mode colour (same as the Atom LED)

    std::array<bool, 6> moving{};    // joints being jogged
    std::array<bool, 6> atLimit{};   // joints stopped at a soft limit
    bool homing = false;
    bool feedbackOk = true;          // false after a failed angle read
};

class TerminalDisplay {
public:
    // enabled = false keeps plain-text output (useful with --debug).
    explicit TerminalDisplay(bool enabled = true,
                             std::chrono::milliseconds refresh = std::chrono::milliseconds(100));
    ~TerminalDisplay();   // restores the normal terminal screen

    TerminalDisplay(const TerminalDisplay&) = delete;
    TerminalDisplay& operator=(const TerminalDisplay&) = delete;

    // True if the full-screen display is being drawn.
    bool active() const { return active_; }

    // Add a line to the message log (or print it, if the display is inactive).
    // Repeated identical messages are shown once with a counter.
    void addMessage(const std::string& text) noexcept;

    // Redraw the screen, at most once per refresh period unless force = true.
    void show(const Status& status, bool force = false) noexcept;

private:
    static constexpr size_t MAX_MESSAGES = 6;

    bool active_ = false;
    std::chrono::milliseconds refresh_;
    std::chrono::steady_clock::time_point lastDraw_{};
    std::deque<std::pair<std::string, int>> messages_;   // text, repeat count

    void render(const Status& status);
    static void writeAll(const std::string& text);
};

}  // namespace ui

