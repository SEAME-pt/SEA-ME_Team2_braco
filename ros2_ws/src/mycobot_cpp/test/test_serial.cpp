// test_serial.cpp - GoogleTests for the serial protocol of mycobot::MyCobot280 (see mycobot.hpp),
// against a pseudo-terminal instead of the arm: the test holds the master side, the class opens
// the slave side as its port (PtyArm, pty_arm.hpp). Checks the frames it sends and how it parses the
// replies.

#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "mycobot.hpp"
#include "pty_arm.hpp"

namespace mc = mycobot;
using mc::fake::appendInt16;
using mc::fake::Bytes;
using mc::fake::frame;

namespace {

class FakeArm : public ::testing::Test {
protected:
    void SetUp() override { arm_ = std::make_unique<mc::MyCobot280>(pty_.path()); }

    Bytes sent(std::chrono::milliseconds timeout = std::chrono::milliseconds(200)) {
        return pty_.sent(timeout);
    }
    void respond(const Bytes& reply, int count = 1) { pty_.respond(reply, count); }

    mc::fake::PtyArm pty_;  // declared first: outlives the arm object
    std::unique_ptr<mc::MyCobot280> arm_;
};

}  // namespace

// ---- Frames sent ----------------------------------------------------------------

TEST_F(FakeArm, CommandWithoutData) {
    arm_->powerOn();
    EXPECT_EQ(sent(), frame(mc::protocol::POWER_ON));
    arm_->stop();
    EXPECT_EQ(sent(), frame(mc::protocol::STOP));
    arm_->powerOff();
    EXPECT_EQ(sent(), frame(mc::protocol::POWER_OFF));
    arm_->releaseAllServos();
    EXPECT_EQ(sent(), frame(mc::protocol::RELEASE_ALL_SERVOS));
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
    pty_.wait();

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
    pty_.wait();

    respond(frame(mc::protocol::GET_SERVO_DATA, {0x01, 0x02}));  // 16-bit value
    EXPECT_EQ(arm_->getServoData(4, mc::servo::ADDR_P, 1), 0x0102);
    pty_.wait();

    respond(frame(mc::protocol::GET_SERVO_DATA, {4, mc::servo::ADDR_P, 26}));  // echo + value
    EXPECT_EQ(arm_->getServoData(4, mc::servo::ADDR_P, 1), 26);
    pty_.wait();

    respond(
        frame(mc::protocol::GET_SERVO_DATA, {5, mc::servo::ADDR_P, 26}));  // echo of another joint
    EXPECT_FALSE(arm_->getServoData(4, mc::servo::ADDR_P, 1).has_value());
    pty_.wait();

    respond(frame(mc::protocol::GET_SERVO_DATA, {4, mc::servo::ADDR_P, 26, 0}));  // unknown format
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

// ---- Served arm (PtyArm::serve) -----------------------------------------------------

TEST_F(FakeArm, ServedArmFollowsTheCommandedPose) {
    pty_.serve();
    EXPECT_EQ(arm_->getAngles(1), mc::ZERO_ANGLES);
    arm_->sendAngles(mc::PARK_ANGLES, mc::MOVE_SPEED);
    ASSERT_TRUE(pty_.waitFor(mc::protocol::SEND_ANGLES, std::chrono::milliseconds(1000)));
    EXPECT_EQ(arm_->getAngles(1), mc::PARK_ANGLES);
    EXPECT_FALSE(arm_->sendAngle(1, -30, 40));
    EXPECT_EQ(arm_->getAngles(1)->at(0), -30);
}

TEST_F(FakeArm, ServedArmReportsItsState) {
    pty_.update([](auto& s) {
        s.temps = {40, 41, 42, 70, 44, 45};
        s.decivolts = {75, 75, 75, 75, 75, 75};
        s.powered = false;
    });
    pty_.serve();
    const auto temps = arm_->checkTemps(1);
    ASSERT_TRUE(temps.has_value());
    EXPECT_EQ(temps->hottestJoint, 4);
    EXPECT_NEAR(arm_->getVoltages(1)->at(5), 7.5, 1e-9);
    EXPECT_EQ(arm_->isPowerOn(), std::optional<bool>(false));
    arm_->powerOn();
    EXPECT_EQ(arm_->isPowerOn(), std::optional<bool>(true));
}

TEST_F(FakeArm, SetPidWritesOnlyTheChangedGainAndReadsItBack) {
    pty_.serve();
    EXPECT_EQ(arm_->getPid(2)->p, mc::FACTORY_PID[1].p);
    EXPECT_TRUE(arm_->setPid(2, {40, 0, 8}));  // P 32 -> 40, I and D unchanged
    EXPECT_EQ(pty_.received(mc::protocol::SET_SERVO_DATA).size(), 1u);
    EXPECT_EQ(arm_->getPid(2)->p, 40);
}

TEST_F(FakeArm, SetPidRefusesWhileTheArmMoves) {
    pty_.update([](auto& s) { s.moving = true; });
    pty_.serve();
    EXPECT_THROW(arm_->setPid(2, {40, 0, 8}), std::runtime_error);
    EXPECT_TRUE(pty_.received(mc::protocol::SET_SERVO_DATA).empty());
}

TEST_F(FakeArm, SetPidFailsWhenTheValueDoesNotStick) {
    pty_.update([](auto& s) { s.ignoreWrites = true; });
    pty_.serve();
    EXPECT_FALSE(arm_->setPid(2, {40, 0, 8}));
    EXPECT_EQ(pty_.received(mc::protocol::SET_SERVO_DATA).size(), 3u);  // SERVO_WRITE_ATTEMPTS
}

// ---- Opening the port -------------------------------------------------------------

TEST(Port, MissingDeviceThrows) {
    EXPECT_THROW(mc::MyCobot280("/dev/does-not-exist"), std::runtime_error);
}

TEST(Port, UnsupportedBaudThrows) {
    EXPECT_THROW(mc::MyCobot280("/dev/null", 9600), std::invalid_argument);
}

TEST(Port, OtherSupportedBaud) {
    mc::fake::PtyArm pty;
    EXPECT_NO_THROW(mc::MyCobot280(pty.path(), 115200));
}

TEST(Port, NotATerminalThrows) {
    try {
        mc::MyCobot280 arm("/dev/null");
        FAIL() << "no exception";
    } catch (const std::runtime_error& e) {
        EXPECT_STREQ(e.what(), "tcgetattr failed on /dev/null: Inappropriate ioctl for device");
    }
}

TEST(Port, WriteAfterTheOtherSideClosedThrows) {
    int master = -1;
    int slave = -1;
    char name[128];
    ASSERT_EQ(openpty(&master, &slave, name, nullptr, nullptr), 0);
    mc::MyCobot280 arm(name);
    close(slave);
    close(master);  // the "cable" is gone: writes fail with EIO
    EXPECT_THROW(arm.stop(), std::runtime_error);
}
