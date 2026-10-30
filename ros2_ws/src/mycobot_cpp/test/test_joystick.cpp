// test_joystick.cpp - GoogleTests for input::Joystick (joystick.hpp). The device is a FIFO: the
// test writes the kernel's js_event structs into it, as /dev/input/js0 would deliver them.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "joystick.hpp"

namespace {

class Fifo : public ::testing::Test {
protected:
    void SetUp() override {
        char dir[] = "/tmp/joystickXXXXXX";
        ASSERT_NE(mkdtemp(dir), nullptr);
        dir_ = dir;
        path_ = dir_ + "/js0";
        ASSERT_EQ(mkfifo(path_.c_str(), 0600), 0);
    }

    void TearDown() override {
        if (writer_ >= 0) close(writer_);
        unlink(path_.c_str());
        rmdir(dir_.c_str());
    }

    // Opened after the Joystick: a FIFO opened for writing needs a reader first.
    void send(uint8_t type, uint8_t number, int16_t value, uint32_t time = 0) {
        if (writer_ < 0) writer_ = open(path_.c_str(), O_WRONLY | O_NONBLOCK);
        ASSERT_GE(writer_, 0);
        const js_event e{time, value, type, number};
        ASSERT_EQ(write(writer_, &e, sizeof(e)), static_cast<ssize_t>(sizeof(e)));
    }

    std::string dir_;
    std::string path_;
    int writer_ = -1;
};

}  // namespace

TEST(Joystick, MissingDeviceThrows) {
    try {
        input::Joystick js("/dev/does-not-exist");
        FAIL() << "no exception";
    } catch (const std::runtime_error& e) {
        EXPECT_STREQ(e.what(), "could not open /dev/does-not-exist: No such file or directory");
    }
}

TEST_F(Fifo, NothingPendingGivesNoEvents) {
    input::Joystick js(path_);
    EXPECT_TRUE(js.readEvents().empty());  // no writer yet: end of file
    send(JS_EVENT_BUTTON, 0, 1);
    js.readEvents();
    EXPECT_TRUE(js.readEvents().empty());  // writer open, nothing written: would block
}

TEST_F(Fifo, ButtonAndAxisEventsInOrder) {
    input::Joystick js(path_);
    send(JS_EVENT_BUTTON, 3, 1, 1000);
    send(JS_EVENT_AXIS, 1, -32767, 1010);
    const auto events = js.readEvents();
    ASSERT_EQ(events.size(), 2u);
    EXPECT_EQ(events[0].type, input::TYPE_BUTTON);
    EXPECT_EQ(events[0].number, 3);
    EXPECT_EQ(events[0].value, 1);
    EXPECT_EQ(events[0].timeMs, 1000u);
    EXPECT_EQ(events[1].type, input::TYPE_AXIS);
    EXPECT_EQ(events[1].number, 1);
    EXPECT_EQ(events[1].value, -32767);
}

TEST_F(Fifo, InitialStateEventsAreSkipped) {
    input::Joystick js(path_);
    send(JS_EVENT_BUTTON | JS_EVENT_INIT, 0, 0);
    send(JS_EVENT_AXIS | JS_EVENT_INIT, 0, 0);
    send(JS_EVENT_BUTTON, 2, 1);
    const auto events = js.readEvents();
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].number, 2);
}

TEST(Joystick, ReadErrorThrows) {
    input::Joystick js("/tmp");  // opens, but read() fails with EISDIR, like an unplugged device
    EXPECT_THROW(js.readEvents(), std::runtime_error);
}
