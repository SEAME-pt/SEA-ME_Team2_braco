// test_display.cpp - GoogleTests for ui::TerminalDisplay (display.hpp). It draws only on a
// terminal, so the tests put stdout on a pseudo-terminal and read the screen from its master side.

#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "display.hpp"

namespace {

using namespace std::chrono_literals;

// |----+----o----| as positionBar() draws it: 'o' at index `at`, '+' (0 deg) at `zero` (-1: none).
std::string bar(int at, int zero = -1) {
    std::string b(25, '-');
    if (zero >= 0) b[zero] = '+';
    b[at] = 'o';
    return "|" + b + "|";
}

ui::Status status() {
    ui::Status s;
    s.limits.fill({-100.0, 100.0});
    s.angles = std::array<double, 6>{0, -100, 50, 100, 0, 0};
    s.modeName = "medium";
    s.speedCap = 50;
    s.maxCoast = 4;
    s.r = 1;
    s.g = 2;
    s.b = 3;
    return s;
}

class Terminal : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_EQ(openpty(&master_, &slave_, nullptr, nullptr, nullptr), 0);
        std::fflush(stdout);
        saved_ = dup(STDOUT_FILENO);
        dup2(slave_, STDOUT_FILENO);
        reader_ = std::thread([this] {
            pollfd p{master_, POLLIN, 0};
            while (!quit_) {
                if (poll(&p, 1, 20) <= 0) continue;
                char buf[4096];
                const ssize_t n = read(master_, buf, sizeof(buf));
                if (n <= 0) continue;
                std::lock_guard<std::mutex> lock(mutex_);
                screen_.append(buf, static_cast<size_t>(n));
            }
        });
    }

    void TearDown() override {
        restore();
        quit_ = true;
        reader_.join();
        close(slave_);
        close(master_);
    }

    void restore() {
        if (saved_ < 0) return;
        std::fflush(stdout);
        dup2(saved_, STDOUT_FILENO);
        close(saved_);
        saved_ = -1;
    }

    // Everything written since the last call, once nothing more arrives for 50 ms.
    std::string output() {
        std::fflush(stdout);
        size_t size = 0;
        do {
            size = length();
            std::this_thread::sleep_for(50ms);
        } while (length() != size);
        std::lock_guard<std::mutex> lock(mutex_);
        return std::exchange(screen_, {});
    }

    size_t length() {
        std::lock_guard<std::mutex> lock(mutex_);
        return screen_.size();
    }

    static bool has(const std::string& text, const std::string& part) {
        return text.find(part) != std::string::npos;
    }

    int master_ = -1;
    int slave_ = -1;
    int saved_ = -1;
    std::thread reader_;
    std::atomic<bool> quit_{false};
    std::mutex mutex_;
    std::string screen_;
};

}  // namespace

// ---- Not drawing ---------------------------------------------------------------------

TEST(TerminalDisplay, NotATerminalPrintsPlainLines) {
    testing::internal::CaptureStdout();
    {
        ui::TerminalDisplay display(true);  // enabled, but stdout is a file here
        EXPECT_FALSE(display.active());
        display.addMessage("hello");
        display.show(status(), true);
    }
    EXPECT_EQ(testing::internal::GetCapturedStdout(), "hello\n");
}

TEST_F(Terminal, DisabledPrintsPlainLines) {
    {
        ui::TerminalDisplay display(false);
        EXPECT_FALSE(display.active());
        display.addMessage("hello");
        display.show(status(), true);
    }
    EXPECT_EQ(output(), "hello\r\n");  // the terminal turns \n into \r\n
}

// ---- Drawing ------------------------------------------------------------------------

TEST_F(Terminal, ScreenIsOpenedAndRestored) {
    auto display = std::make_unique<ui::TerminalDisplay>(true);
    EXPECT_TRUE(display->active());
    EXPECT_TRUE(has(output(), "\x1b[?1049h"));  // alternate screen on
    display.reset();
    EXPECT_TRUE(has(output(), "\x1b[?1049l"));  // and off again
}

TEST_F(Terminal, DrawsModeJointsAndBars) {
    ui::TerminalDisplay display(true);
    output();
    ui::Status s = status();
    s.moving[2] = true;
    s.atLimit[1] = true;
    display.show(s, true);
    const std::string screen = output();

    EXPECT_TRUE(has(screen, "myCobot 280 - joystick control"));
    EXPECT_TRUE(has(screen, "\x1b[48;2;1;2;3m"));  // the mode colour swatch
    EXPECT_TRUE(has(screen, "medium   cap  50 %   max coast  4 deg"));
    EXPECT_TRUE(has(screen, "JOGGING\x1b[0m J3"));
    EXPECT_TRUE(has(screen, " J1        0.00      -100 .. 100    " + bar(12)));
    EXPECT_TRUE(has(screen, " J2     -100.00      -100 .. 100    " + bar(0, 12) + "  \x1b[33m"));
    EXPECT_TRUE(has(screen, " J3       50.00      -100 .. 100    " + bar(18, 12) + "  \x1b[1m<"));
    EXPECT_TRUE(has(screen, bar(24, 12)));  // J4 at its max
}

TEST_F(Terminal, ArmStates) {
    ui::TerminalDisplay display(true);
    ui::Status s = status();
    display.show(s, true);
    EXPECT_TRUE(has(output(), "idle"));

    s.homing = true;
    display.show(s, true);
    EXPECT_TRUE(has(output(), "HOMING to 0"));

    s.feedbackOk = false;
    s.angles.reset();
    s.limits[0] = {0.0, 100.0};  // 0 deg is not inside: no '+'
    display.show(s, true);
    const std::string screen = output();
    EXPECT_TRUE(has(screen, "NO ANGLE FEEDBACK"));
    EXPECT_TRUE(has(screen, " J1          --         0 .. 100  "));
    EXPECT_FALSE(has(screen, "|o"));  // no bars without angles
}

TEST_F(Terminal, RedrawIsRateLimited) {
    ui::TerminalDisplay display(true, std::chrono::hours(1));
    display.show(status(), true);
    EXPECT_TRUE(has(output(), "J6"));
    display.show(status());  // within the hour: skipped
    EXPECT_TRUE(output().empty());
}

TEST_F(Terminal, MessagesAreCountedAndTheOldestDropped) {
    ui::TerminalDisplay display(true);
    display.addMessage("joint 1 at its limit");
    display.addMessage("joint 1 at its limit");
    display.show(status(), true);
    EXPECT_TRUE(has(output(), "joint 1 at its limit  (x2)"));

    for (int i = 1; i <= 6; ++i) display.addMessage("message " + std::to_string(i));
    display.show(status(), true);
    const std::string screen = output();
    EXPECT_FALSE(has(screen, "joint 1 at its limit"));  // 7 messages, room for 6
    EXPECT_TRUE(has(screen, "message 1"));
    EXPECT_TRUE(has(screen, "message 6"));
}
