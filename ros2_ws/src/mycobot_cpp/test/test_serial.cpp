// test_serial.cpp - GoogleTests for the serial protocol of mycobot::MyCobot280 (see mycobot.hpp),
// against a pseudo-terminal instead of the arm: the test holds the master side, the class opens
// the slave side as its port. Checks the frames it sends and how it parses the replies.

#include <gtest/gtest.h>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "mycobot.hpp"

namespace mc = mycobot;
using Bytes = std::vector<uint8_t>;

namespace {

// Big-endian int16, as the arm sends it.
void appendInt16(Bytes& out, int value) {
    const uint16_t v = static_cast<uint16_t>(static_cast<int16_t>(value));
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v & 0xFF));
}

Bytes frame(uint8_t cmd, const Bytes& data = {}) {
    Bytes f = {mc::protocol::HEADER, mc::protocol::HEADER, static_cast<uint8_t>(data.size() + 2),
               cmd};
    f.insert(f.end(), data.begin(), data.end());
    f.push_back(mc::protocol::FOOTER);
    return f;
}

class FakeArm : public ::testing::Test {
protected:
    void SetUp() override {
        char name[128];
        ASSERT_EQ(openpty(&master_, &slave_, name, nullptr, nullptr), 0);
        arm_ = std::make_unique<mc::MyCobot280>(name);
    }

    void TearDown() override {
        if (responder_.joinable()) responder_.join();
        arm_.reset();
        close(slave_);
        close(master_);
    }

    // Every byte the arm object wrote, waiting up to `timeout` for the first one.
    Bytes sent(std::chrono::milliseconds timeout = std::chrono::milliseconds(200)) {
        Bytes out;
        pollfd p{master_, POLLIN, 0};
        int wait = static_cast<int>(timeout.count());
        while (poll(&p, 1, wait) > 0) {
            uint8_t buf[256];
            const ssize_t n = read(master_, buf, sizeof(buf));
            if (n <= 0) break;
            out.insert(out.end(), buf, buf + n);
            wait = 20;  // the rest of the frame follows at once
        }
        return out;
    }

    // Answer the next `count` requests with `reply`, from another thread (the class blocks in its read).
    void respond(const Bytes& reply, int count = 1) {
        responder_ = std::thread([this, reply, count] {
            for (int i = 0; i < count; ++i) {
                if (sent(std::chrono::milliseconds(2000)).empty()) return;
                ASSERT_EQ(write(master_, reply.data(), reply.size()),
                          static_cast<ssize_t>(reply.size()));
            }
        });
    }

    int master_ = -1;
    int slave_ = -1;
    std::unique_ptr<mc::MyCobot280> arm_;
    std::thread responder_;
};

}  // namespace

// ---- Frames sent ----------------------------------------------------------------

TEST_F(FakeArm, CommandWithoutData) {
    arm_->powerOn();
    EXPECT_EQ(sent(), frame(mc::protocol::POWER_ON));
    arm_->stop();
    EXPECT_EQ(sent(), frame(mc::protocol::STOP));
}

TEST_F(FakeArm, SendAnglesEncodesHundredthsBigEndian) {
    EXPECT_FALSE(arm_->sendAngles({10, -20, 0, 0, 0, 0.5}, 50));
    Bytes data;
    for (int v : {1000, -2000, 0, 0, 0, 50}) appendInt16(data, v);
    data.push_back(50);
    EXPECT_EQ(sent(), frame(mc::protocol::SEND_ANGLES, data));
}

TEST_F(FakeArm, SendAnglesClampsToJointLimits) {
    EXPECT_TRUE(arm_->sendAngles({200, 0, 0, 0, 0, 0}, 30));
    const Bytes out = sent();
    ASSERT_EQ(out.size(), 18u);
    EXPECT_EQ((out[4] << 8) | out[5], 16600);  // J1 max 166 deg
}

TEST_F(FakeArm, SpeedIsClampedToOneToHundred) {
    arm_->sendAngles(mc::ZERO_ANGLES, 0);
    EXPECT_EQ(sent().at(16), 1);
    arm_->sendAngles(mc::ZERO_ANGLES, 500);
    EXPECT_EQ(sent().at(16), 100);
}

TEST_F(FakeArm, PoseIntoTheTableIsNotSent) {
    EXPECT_TRUE(arm_->sendAngles({0, -130, 0, 0, 0, 0}, 30));
    EXPECT_TRUE(sent().empty());
}

TEST_F(FakeArm, NonFiniteAngleThrowsAndSendsNothing) {
    EXPECT_THROW(arm_->sendAngles({0, NAN, 0, 0, 0, 0}, 30), std::invalid_argument);
    EXPECT_THROW(arm_->sendAngle(1, INFINITY, 30), std::invalid_argument);
    EXPECT_THROW(arm_->sendAngle(7, 0, 30), std::invalid_argument);
    EXPECT_TRUE(sent().empty());
}

TEST_F(FakeArm, SendAngleUsesTheLastCommandedPose) {
    arm_->sendAngles(mc::ZERO_ANGLES, 30);
    sent();
    EXPECT_FALSE(arm_->sendAngle(3, -45.5, 40));
    Bytes data = {3};
    appendInt16(data, -4550);
    data.push_back(40);
    EXPECT_EQ(sent(), frame(mc::protocol::SEND_ANGLE, data));
}

TEST_F(FakeArm, GripperAndColour) {
    arm_->setGripperState(5, 70);  // any non-zero state closes
    EXPECT_EQ(sent(), frame(mc::protocol::SET_GRIPPER_STATE, {1, 70}));
    arm_->setColor(1, 2, 3);
    EXPECT_EQ(sent(), frame(mc::protocol::SET_COLOR, {1, 2, 3}));
}

// ---- Replies parsed -------------------------------------------------------------

TEST_F(FakeArm, GetAnglesParsesTheReply) {
    Bytes data;
    for (int v : {1000, -2000, 4550, 0, -17800, 16600}) appendInt16(data, v);
    respond(frame(mc::protocol::GET_ANGLES, data));
    const auto angles = arm_->getAngles(1);
    ASSERT_TRUE(angles.has_value());
    EXPECT_EQ(*angles, (mc::Angles{10, -20, 45.5, 0, -178, 166}));
}

TEST_F(FakeArm, ReplyIsFoundAfterNoiseAndOtherFrames) {
    Bytes reply = {0x00, 0xFE, 0x13};                         // noise and a half header
    const Bytes other = frame(mc::protocol::IS_MOVING, {1});  // a frame for another command
    reply.insert(reply.end(), other.begin(), other.end());
    const Bytes power = frame(mc::protocol::IS_POWER_ON, {1});
    reply.insert(reply.end(), power.begin(), power.end());
    respond(reply);
    EXPECT_EQ(arm_->isPowerOn(), std::optional<bool>(true));
}

TEST_F(FakeArm, GetCoordsScalesPositionAndRotation) {
    Bytes data;
    for (int v : {1505, -200, 3000, 9000, -4550, 0}) appendInt16(data, v);
    respond(frame(mc::protocol::GET_COORDS, data));
    const auto coords = arm_->getCoords(1);
    ASSERT_TRUE(coords.has_value());
    EXPECT_EQ(*coords, (mc::Coords{150.5, -20, 300, 90, -45.5, 0}));
}

TEST_F(FakeArm, WrongReplySizeGivesNothing) {
    respond(frame(mc::protocol::GET_ANGLES, {0, 1, 2}));
    EXPECT_FALSE(arm_->getAngles(1).has_value());
}

TEST_F(FakeArm, TemperaturesOneByteOrTwoBytesPerJoint) {
    respond(frame(mc::protocol::GET_SERVO_TEMPS, {30, 31, 32, 33, 34, 66}));
    const auto temps = arm_->checkTemps(1);
    ASSERT_TRUE(temps.has_value());
    EXPECT_EQ(temps->temps, (mc::ServoTemps{30, 31, 32, 33, 34, 66}));
    EXPECT_EQ(temps->level, mc::TempLevel::Hot);
    EXPECT_EQ(temps->hottestJoint, 6);
    responder_.join();

    Bytes wide;
    for (int v : {40, 41, 42, 43, 44, 45}) appendInt16(wide, v);
    respond(frame(mc::protocol::GET_SERVO_TEMPS, wide));
    EXPECT_EQ(arm_->getTemps(1), (mc::ServoTemps{40, 41, 42, 43, 44, 45}));
}

TEST_F(FakeArm, UnknownTemperatureFormatGivesNothing) {
    respond(frame(mc::protocol::GET_SERVO_TEMPS, {30, 31, 32}));
    EXPECT_FALSE(arm_->getTemps(1).has_value());
}

TEST_F(FakeArm, VoltagesInTenthsOfAVolt) {
    respond(frame(mc::protocol::GET_SERVO_VOLTAGES, {120, 121, 119, 120, 75, 76}));
    const auto volts = arm_->getVoltages(1);
    ASSERT_TRUE(volts.has_value());
    EXPECT_NEAR((*volts)[0], 12.0, 1e-9);
    EXPECT_NEAR((*volts)[4], 7.5, 1e-9);
}

TEST_F(FakeArm, ServoDataReplyFormats) {
    respond(frame(mc::protocol::GET_SERVO_DATA, {25}));  // value
    EXPECT_EQ(arm_->getServoData(4, mc::servo::ADDR_P, 1), 25);
    responder_.join();

    respond(frame(mc::protocol::GET_SERVO_DATA, {0x01, 0x02}));  // 16-bit value
    EXPECT_EQ(arm_->getServoData(4, mc::servo::ADDR_P, 1), 0x0102);
    responder_.join();

    respond(frame(mc::protocol::GET_SERVO_DATA, {4, mc::servo::ADDR_P, 26}));  // echo + value
    EXPECT_EQ(arm_->getServoData(4, mc::servo::ADDR_P, 1), 26);
    responder_.join();

    respond(
        frame(mc::protocol::GET_SERVO_DATA, {5, mc::servo::ADDR_P, 26}));  // echo of another joint
    EXPECT_FALSE(arm_->getServoData(4, mc::servo::ADDR_P, 1).has_value());
}

TEST_F(FakeArm, NoReplyGivesNothing) {
    EXPECT_FALSE(arm_->isMoving().has_value());  // 5 retries of 500 ms
}

// ---- PID writes -----------------------------------------------------------------

TEST_F(FakeArm, SetPidOutOfBoundsSendsNothing) {
    EXPECT_THROW(arm_->setPid(1, {100, 0, 20}), std::invalid_argument);
    EXPECT_TRUE(sent().empty());
}

TEST_F(FakeArm, SetPidRefusesWhenTheCurrentValuesCannotBeRead) {
    EXPECT_THROW(arm_->setPid(1, mc::FACTORY_PID[0]), std::runtime_error);
    // Only read requests went out (7-byte GET_SERVO_DATA frames), no write.
    const Bytes out = sent();
    ASSERT_FALSE(out.empty());
    ASSERT_EQ(out.size() % 7, 0u);
    for (size_t i = 0; i < out.size(); i += 7) EXPECT_EQ(out[i + 3], mc::protocol::GET_SERVO_DATA);
}

// ---- Opening the port -------------------------------------------------------------

TEST(Port, MissingDeviceThrows) {
    EXPECT_THROW(mc::MyCobot280("/dev/does-not-exist"), std::runtime_error);
}

TEST(Port, UnsupportedBaudThrows) {
    EXPECT_THROW(mc::MyCobot280("/dev/null", 9600), std::invalid_argument);
}
