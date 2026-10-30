// test_lcd_display.cpp - GoogleTests for ui::LcdDisplay (lcd_display.hpp) without the I2C bus.
//
// The bus is a plain file: every register write lands in it as two bytes (register, value). The
// one call a file cannot take, ioctl(I2C_SLAVE) to pick the chip, is answered by the ioctl() below:
// defined in this executable, it takes precedence over the C library's, for this test only. Every
// other request goes to the real ioctl(). The two chips are told apart by the first byte: 0x80 /
// 0x40 are LCD command / character, anything else a backlight register.

#include <dlfcn.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include "display.hpp"
#include "lcd_display.hpp"

namespace {

enum class I2c { Accept, Refuse };
I2c g_i2c = I2c::Accept;  // what the fake ioctl(I2C_SLAVE) does

}  // namespace

// The C library's own signature (variadic), so it replaces it.
extern "C" int ioctl(int fd, unsigned long request, ...) noexcept {
    va_list args;
    va_start(args, request);
    void* arg = va_arg(args, void*);
    va_end(args);
    if (request == I2C_SLAVE) {
        if (g_i2c == I2c::Accept) return 0;
        errno = EIO;
        return -1;
    }
    using Real = int (*)(int, unsigned long, ...);
    static const auto real = reinterpret_cast<Real>(dlsym(RTLD_NEXT, "ioctl"));
    return real(fd, request, arg);
}

namespace {

using ui::LcdDisplay;
using Write = std::pair<uint8_t, uint8_t>;  // register, value

constexpr uint8_t CMD = 0x80;
constexpr uint8_t DATA = 0x40;

ui::Status idle() {
    ui::Status s;
    s.angles = std::array<double, 6>{0, 0, 0, 0, 0, 0};
    s.temps = std::array<int, 6>{31, 33, 35, 41, 38, 30};
    return s;
}

// An LcdDisplay on a temporary file, and the writes it made since the last call to writes().
class FileBus : public ::testing::Test {
protected:
    void SetUp() override {
        g_i2c = I2c::Accept;
        char name[] = "/tmp/lcd_busXXXXXX";
        fd_ = mkstemp(name);
        ASSERT_GE(fd_, 0);
        path_ = name;
    }

    void TearDown() override {
        g_i2c = I2c::Accept;
        close(fd_);
        unlink(path_.c_str());
    }

    std::vector<Write> writes() {
        std::vector<Write> out;
        uint8_t pair[2];
        while (pread(fd_, pair, 2, static_cast<off_t>(read_)) == 2) {
            out.emplace_back(pair[0], pair[1]);
            read_ += 2;
        }
        return out;
    }

    // The characters written, and the number of LCD commands among the writes.
    static std::string text(const std::vector<Write>& w) {
        std::string out;
        for (const auto& [reg, value] : w) {
            if (reg == DATA) out += static_cast<char>(value);
        }
        return out;
    }
    static size_t commands(const std::vector<Write>& w) {
        size_t n = 0;
        for (const auto& [reg, value] : w) n += reg == CMD;
        return n;
    }
    static size_t backlight(const std::vector<Write>& w) {
        return w.size() - text(w).size() - commands(w);
    }

    int fd_ = -1;
    std::string path_;
    size_t read_ = 0;
};

}  // namespace

// ---- What is shown (linesFor, no bus) -------------------------------------------------

TEST(LcdLines, IdleShowsTheTemperatures) {
    EXPECT_EQ(LcdDisplay::linesFor(idle()),
              (std::array<std::string, 2>{"1:31  2:33  3:35", "4:41  5:38  6:30"}));
}

TEST(LcdLines, TemperaturesKeepTwoDigits) {
    ui::Status s = idle();
    s.temps = std::array<int, 6>{-5, 100, 7, 0, 0, 0};
    EXPECT_EQ(LcdDisplay::linesFor(s)[0], "1: 0  2:99  3: 7");
}

TEST(LcdLines, AnglesWhenTheTemperaturesAreUnknown) {
    ui::Status s;
    s.angles = std::array<double, 6>{-150, -12.4, 128, 0.6, 168, -168};
    EXPECT_EQ(LcdDisplay::linesFor(s),
              (std::array<std::string, 2>{" -150  -12  128 ", "    1  168 -168 "}));
    s.angles.reset();
    EXPECT_EQ(LcdDisplay::linesFor(s)[0], "ANGLES UNKNOWN  ");
    EXPECT_EQ(LcdDisplay::linesFor(s)[1], std::string(16, ' '));
}

TEST(LcdLines, MovingJointsAreCentred) {
    ui::Status s = idle();
    s.moving[0] = s.moving[2] = true;
    EXPECT_EQ(LcdDisplay::linesFor(s),
              (std::array<std::string, 2>{"ARM MOVING...   ", "     J1,J3      "}));
    s.moving.fill(true);  // "J1,J2,J3,J4,J5,J6" is 17 characters: cut, not centred
    EXPECT_EQ(LcdDisplay::linesFor(s)[1], "J1,J2,J3,J4,J5,J");
    s.homing = true;
    EXPECT_EQ(LcdDisplay::linesFor(s)[1], "   HOMING ALL   ");
}

TEST(LcdLines, HotServoOverridesMoving) {
    ui::Status s = idle();
    s.moving[1] = true;
    s.hotJoint = 4;
    EXPECT_EQ(LcdDisplay::linesFor(s),
              (std::array<std::string, 2>{"SERVO HOT! J4   ", "41 C - rest arm "}));
    s.autoRest = true;
    EXPECT_EQ(LcdDisplay::linesFor(s),
              (std::array<std::string, 2>{"REST: SERVO HOT ", "J4 41 C cooling "}));
}

TEST(LcdLines, NoFeedbackOverridesEverything) {
    ui::Status s = idle();
    s.hotJoint = 4;
    s.feedbackOk = false;
    EXPECT_EQ(LcdDisplay::linesFor(s),
              (std::array<std::string, 2>{"NO FEEDBACK     ", "check arm power "}));
}

// ---- Opening the bus ------------------------------------------------------------------

TEST(LcdOpen, MissingBus) {
    LcdDisplay lcd("/dev/does-not-exist");
    EXPECT_FALSE(lcd.ok());
    EXPECT_EQ(lcd.error(), "/dev/does-not-exist: No such file or directory");
    lcd.show(idle(), true);  // nothing to draw on, no crash
}

TEST_F(FileBus, ChipThatDoesNotAnswer) {
    g_i2c = I2c::Refuse;
    LcdDisplay lcd(path_);
    EXPECT_FALSE(lcd.ok());
    EXPECT_EQ(lcd.error(), "no answer from the LCD on " + path_ + " (check wiring and power)");
    EXPECT_TRUE(writes().empty());
}

TEST(LcdOpen, WritesThatFail) {
    g_i2c = I2c::Accept;
    LcdDisplay lcd("/dev/full");  // every write fails with ENOSPC
    EXPECT_FALSE(lcd.ok());
    EXPECT_NE(lcd.error().find("no answer"), std::string::npos);
}

TEST_F(FileBus, InitialisesBacklightThenLcd) {
    LcdDisplay lcd(path_);
    ASSERT_TRUE(lcd.ok()) << lcd.error();
    const auto w = writes();
    const std::vector<Write> expected = {
        {0x00, 0x00}, {0x01, 0x00}, {0x08, 0xAA}, {0x02, 0xFF}, {0x03, 0xFF}, {0x04, 0xFF},
        {0x05, 0xFF}, {CMD, 0x38},  {CMD, 0x39},  {CMD, 0x14},  {CMD, 0x74},  {CMD, 0x54},
        {CMD, 0x6F},  {CMD, 0x38},  {CMD, 0x0C},  {CMD, 0x01},  {CMD, 0x06}};
    EXPECT_EQ(w, expected);
}

// ---- Drawing --------------------------------------------------------------------------

TEST_F(FileBus, WritesOnlyWhatChanged) {
    LcdDisplay lcd(path_, std::chrono::hours(1));
    ASSERT_TRUE(lcd.ok());
    writes();

    ui::Status s = idle();
    s.r = 255;  // white after init: the colour differs
    s.g = s.b = 0;
    lcd.show(s, true);
    auto w = writes();
    EXPECT_EQ(text(w), "1:31  2:33  3:354:41  5:38  6:30");
    EXPECT_EQ(commands(w), 2u);  // one cursor move per line, then the LCD advances by itself
    EXPECT_EQ(backlight(w), 3u);

    lcd.show(s, true);  // nothing changed
    EXPECT_TRUE(writes().empty());

    s.temps->at(4) = 39;  // "5:38" -> "5:39": one character
    lcd.show(s);          // inside the refresh period: not drawn
    EXPECT_TRUE(writes().empty());
    lcd.show(s, true);
    w = writes();
    EXPECT_EQ(text(w), "9");
    EXPECT_EQ(w.front(), (Write{CMD, 0xC0 | 9}));  // row 1, column 9
}

TEST_F(FileBus, FailedLineIsRewrittenInFull) {
    LcdDisplay lcd(path_, std::chrono::milliseconds(0));
    ASSERT_TRUE(lcd.ok());
    ui::Status s = idle();
    s.r = 1;
    lcd.show(s, true);  // ends on the backlight chip
    writes();

    s.temps->at(0) = 32;
    g_i2c = I2c::Refuse;  // selecting the LCD chip again fails
    lcd.show(s, true);
    EXPECT_TRUE(writes().empty());

    g_i2c = I2c::Accept;
    lcd.show(s, true);
    EXPECT_EQ(text(writes()), "1:32  2:33  3:35");  // the whole line: its content was unknown
}

TEST_F(FileBus, FailedBacklightIsSetAgain) {
    LcdDisplay lcd(path_, std::chrono::milliseconds(0));
    ASSERT_TRUE(lcd.ok());
    ui::Status s = idle();
    lcd.show(s, true);  // text and backlight: ends on the backlight chip
    s.temps->at(0) = 32;
    lcd.show(s, true);  // text only: ends on the LCD chip
    writes();

    s.g = 255;
    g_i2c = I2c::Refuse;  // selecting the backlight chip fails
    lcd.show(s, true);
    EXPECT_TRUE(writes().empty());

    g_i2c = I2c::Accept;
    lcd.show(s, true);
    EXPECT_EQ(backlight(writes()), 3u);
    lcd.show(s, true);  // now known: not written again
    EXPECT_TRUE(writes().empty());
}
