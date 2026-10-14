// mycobot.hpp - C++ interface to the myCobot 280 (Jetson Nano) over serial.
//
// Speaks the same serial protocol as pymycobot, so programs need neither
// Python nor pymycobot. The implementation lives in mycobot.cpp.
//
// Frame format: 0xFE 0xFE <len> <command> <data...> 0xFA
//               len = data size + 2 (command byte + footer byte)
//
// Units (same as pymycobot):
//   angles  degrees, sent as int16 x 100
//   coords  X/Y/Z in mm (int16 x 10), RX/RY/RZ in degrees (int16 x 100)
//   speed   1..100 (percent)

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mycobot {

using Angles = std::array<double, 6>;  // J1..J6 in degrees
using Coords = std::array<double, 6>;  // X, Y, Z (mm), RX, RY, RZ (deg)

struct JointLimit {
    double min;   // degrees
    double max;   // degrees
};
using JointLimits = std::array<JointLimit, 6>;   // index 0 = J1 ... 5 = J6

// Limits enforced by the arm's firmware, read from this arm with
// get_joint_min_angle() / get_joint_max_angle(). Absolute ceiling: soft
// limits can never be set outside these.
constexpr JointLimits FIRMWARE_LIMITS = {{
    {-168.0, 168.0},   // J1
    {-140.0, 140.0},   // J2
    {-150.0, 150.0},   // J3
    {-150.0, 150.0},   // J4
    {-155.0, 160.0},   // J5
    {-180.0, 180.0},   // J6
}};

// Default soft limits for this setup (from the joystick program). Tighter
// than the firmware to keep the arm away from collisions with itself, the
// table and cables. Change per setup with setSoftLimits().
constexpr JointLimits DEFAULT_SOFT_LIMITS = {{
    {-160.0, 160.0},   // J1
    {-130.0, 130.0},   // J2
    {-140.0, 140.0},   // J3
    {-120.0, 120.0},   // J4
    {-135.0, 135.0},   // J5
    {-170.0, 170.0},   // J6
}};

// Command codes, taken from pymycobot's ProtocolCode.
// Verify against your installed version:
//   grep -nE "= 0x" ~/.local/lib/python3.8/site-packages/pymycobot/common.py
namespace protocol {
constexpr uint8_t HEADER = 0xFE;
constexpr uint8_t FOOTER = 0xFA;

constexpr uint8_t POWER_ON = 0x10;
constexpr uint8_t POWER_OFF = 0x11;
constexpr uint8_t IS_POWER_ON = 0x12;
constexpr uint8_t RELEASE_ALL_SERVOS = 0x13;

constexpr uint8_t GET_ANGLES = 0x20;
constexpr uint8_t SEND_ANGLE = 0x21;
constexpr uint8_t SEND_ANGLES = 0x22;
constexpr uint8_t GET_COORDS = 0x23;
constexpr uint8_t STOP = 0x29;
constexpr uint8_t IS_MOVING = 0x2B;

constexpr uint8_t SET_GRIPPER_STATE = 0x66;
constexpr uint8_t SET_COLOR = 0x6A;
}  // namespace protocol

class MyCobot280 {
public:
    static constexpr const char* DEFAULT_PORT = "/dev/ttyTHS1";
    static constexpr int DEFAULT_BAUD = 1000000;

    // Opens and configures the serial port. Throws std::runtime_error on failure.
    explicit MyCobot280(const std::string& port = DEFAULT_PORT, int baud = DEFAULT_BAUD);
    ~MyCobot280();

    // One object owns the port: no copies.
    MyCobot280(const MyCobot280&) = delete;
    MyCobot280& operator=(const MyCobot280&) = delete;

    // ---- Power -------------------------------------------------------------
    void powerOn();                       // lock all joints (torque on)
    void powerOff();                      // power off the servos
    void releaseAllServos();              // free all joints (move by hand)
    std::optional<bool> isPowerOn();      // nullopt if no valid reply

    // ---- Joint limits --------------------------------------------------------
    // Every angle sent to the arm is clamped to the soft limits, so no program
    // can command a joint outside them. Throws std::invalid_argument if any
    // limit lies outside FIRMWARE_LIMITS or has min >= max.
    void setSoftLimits(const JointLimits& limits);
    const JointLimits& softLimits() const { return softLimits_; }

    // ---- Motion (fire-and-forget, like pymycobot) --------------------------
    // Angles outside the soft limits are clamped to the nearest limit.
    // Returns true if any angle was clamped. Throws std::invalid_argument for
    // non-finite angles (NaN/inf) or a joint outside 1..6.
    bool sendAngles(const Angles& angles, int speed);
    bool sendAngle(int joint, double angle, int speed);   // joint 1..6
    void stop();
    std::optional<bool> isMoving();       // nullopt if no valid reply

    // ---- Feedback (nullopt if no valid reply after all retries) ------------
    std::optional<Angles> getAngles(int retries = 5);
    std::optional<Coords> getCoords(int retries = 5);

    // ---- Gripper and Atom LED ----------------------------------------------
    void setGripperState(int state, int speed);   // 0 = open, 1 = close
    void setColor(uint8_t r, uint8_t g, uint8_t b);

private:
    int fd_ = -1;
    JointLimits softLimits_ = DEFAULT_SOFT_LIMITS;

    // Clamp one angle to the soft limit of `joint` (1..6); sets `clamped`.
    double limitAngle(int joint, double angle, bool& clamped) const;

    // Build a frame and write it to the port.
    void sendCommand(uint8_t cmd, const std::vector<uint8_t>& data = {});

    // Read until a complete frame for `cmd` arrives; return its data bytes.
    std::optional<std::vector<uint8_t>> readReply(uint8_t cmd,
                                                  std::chrono::milliseconds timeout);

    // Send a request and wait for a reply with exactly `expected_size` data
    // bytes, retrying up to `retries` times.
    std::optional<std::vector<uint8_t>> query(uint8_t cmd,
                                              size_t expected_size,
                                              int retries = 5);

    static void appendInt16(std::vector<uint8_t>& out, int16_t value);  // big-endian
    static int16_t readInt16(const std::vector<uint8_t>& data, size_t offset);
    static uint8_t clampSpeed(int speed);                                // 1..100
};

}  // namespace mycobot
