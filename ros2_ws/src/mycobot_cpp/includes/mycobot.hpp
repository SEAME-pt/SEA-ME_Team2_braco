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
    double min;  // degrees
    double max;  // degrees
};
using JointLimits = std::array<JointLimit, 6>;  // index 0 = J1 ... 5 = J6

// The one set of joint limits. Based on the limits enforced by the arm's
// firmware (read from this arm with get_joint_min_angle() / get_joint_max_angle()
// and equal to Elephant Robotics' official URDF), minus 1-2 deg so a joint
// never rests on its mechanical end stop. The park pose is inside them.
// A box of per-joint limits cannot keep the tool off the table (that depends on
// J2..J5 together), so poses are also checked with minHeightMm() below.
constexpr JointLimits JOINT_LIMITS = {{
    {-166.0, 166.0},  // J1
    {-138.0, 138.0},  // J2
    {-149.0, 149.0},  // J3
    {-149.0, 149.0},  // J4
    {-153.0, 158.0},  // J5
    {-178.0, 178.0},  // J6
}};

// Fixed poses, shared by zero, park and the ROS 2 nodes of braco_bringup.
// Park folds the arm (wrist 160 mm above the table); J3 -148 is just inside its limit.
constexpr Angles ZERO_ANGLES = {0, 0, 0, 0, 0, 0};
constexpr Angles PARK_ANGLES = {0, 131, -148, 146, 4.5, 0};

// Speed (1..100) and arrival tolerance used by every zero / park move, stand-alone and ROS 2.
constexpr int MOVE_SPEED = 30;
constexpr double POSE_TOLERANCE_DEG = 2.0;

// One bounded step from `current` toward `pose`: every joint moves the same fraction, so
// they all arrive together, and no joint ends more than about `lead_deg` short of the
// worst deviation. Returns nothing when every joint is within `tolerance_deg` of `pose`.
std::optional<Angles> stepToward(const Angles& current, const Angles& pose, double lead_deg,
                                 double tolerance_deg);

// Table clearance, from the measured setup. Heights are above the table.
constexpr double BASE_HEIGHT_MM = 140.0;    // table -> top of the Jetson casing, G-base included
constexpr double CASING_HEIGHT_MM = 109.7;  // Jetson casing, bottom -> top (vendor mesh); the
                                            // kinematics start at its bottom, 30.3 mm up
constexpr double TOOL_LENGTH_MM = 130.0;    // J6 flange -> tip of camera + gripper
                                            // (130 closed, 110 open: use the longer)
constexpr double TABLE_MARGIN_MM = 20.0;    // lowest point must stay above this

// Height above the table (mm) of the lowest of the wrist (J4, J5, J6) and the
// tool tip, for these joint angles. Forward kinematics of the myCobot 280 Jetson
// Nano, the same joint origins as braco_description (test_urdf.py checks them). Only height is checked: the casing footprint and the
// sideways size of the gripper are not modelled.
double minHeightMm(const Angles& angles);

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

// Servo configuration. VERIFY these two before using the PID methods:
//   grep -nE "SERVO_DATA" ~/.local/lib/python3.8/site-packages/pymycobot/common.py
constexpr uint8_t SET_SERVO_DATA = 0x52;
constexpr uint8_t GET_SERVO_DATA = 0x53;

// Servo status (read-only). VERIFY against pymycobot:
//   grep -nE "SERVO_TEMPS" ~/.local/lib/python3.8/site-packages/pymycobot/common.py
constexpr uint8_t GET_SERVO_VOLTAGES = 0xE3;  // pymycobot ProtocolCode.GET_SERVO_VOLTAGES
constexpr uint8_t GET_SERVO_TEMPS = 0xE5;
}  // namespace protocol

// ---- Servo PID ---------------------------------------------------------------
// Servo data addresses of the position-loop gains (see the Motor PID docs).
namespace servo {
constexpr uint8_t ADDR_P = 21;
constexpr uint8_t ADDR_D = 22;
constexpr uint8_t ADDR_I = 23;
}  // namespace servo

struct Pid {
    int p;
    int i;
    int d;
};
using PidSet = std::array<Pid, 6>;  // index 0 = J1 ... 5 = J6

// ---- Servo temperatures --------------------------------------------------------
using ServoTemps = std::array<int, 6>;     // J1..J6 servo temperature in deg C
using ServoVolts = std::array<double, 6>;  // J1..J6 servo supply voltage in V

// The arm reports each voltage as one byte in steps of 0.1 V (pymycobot: "volts < 24 V").
constexpr double VOLTAGE_STEP_V = 0.1;

// The servos protect themselves from overheating at around 70 deg C; these
// thresholds warn well before that.
constexpr int TEMP_WARM_C = 55;  // getting warm: let the arm rest soon
constexpr int TEMP_HOT_C = 65;   // close to the servos' own protection

enum class TempLevel { Normal, Warm, Hot };

struct TempCheck {
    ServoTemps temps;  // all six temperatures
    TempLevel level;   // level of the hottest joint
    int hottestJoint;  // 1..6
};

// The hottest joint of `temps` sets the level (same check as MyCobot280::checkTemps()).
TempCheck evaluateTemps(const ServoTemps& temps);

// Backlight / LED colour for each level: green normal, yellow warm, red hot.
struct Rgb {
    uint8_t r, g, b;
};
Rgb tempColour(TempLevel level);

// Factory values read from this arm with pymycobot (get_servo_data 21/23/22).
constexpr PidSet FACTORY_PID = {{
    {15, 0, 20},  // J1
    {32, 0, 8},   // J2
    {32, 0, 8},   // J3
    {25, 0, 25},  // J4
    {25, 0, 25},  // J5
    {25, 0, 25},  // J6
}};

// Safety bounds for setPid(): P and D at most 1.5 x factory, I at most 8.
constexpr double PID_MAX_FACTOR = 1.5;
constexpr int PID_MAX_I = 8;

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
    void powerOn();                   // lock all joints (torque on)
    void powerOff();                  // power off the servos
    void releaseAllServos();          // free all joints (move by hand)
    std::optional<bool> isPowerOn();  // nullopt if no valid reply

    // ---- Motion (fire-and-forget, like pymycobot) --------------------------
    // Angles outside JOINT_LIMITS are clamped to the nearest limit. A pose whose
    // lowest point is under TABLE_MARGIN_MM is not sent at all (the constraint
    // involves several joints, so there is nothing to clamp to).
    // Returns true if any angle was clamped or the move was refused. Throws
    // std::invalid_argument for non-finite angles (NaN/inf) or a joint outside 1..6.
    // sendAngle() checks the height with the last known angles (from getAngles()
    // or sendAngles()), reading them once if there are none yet.
    bool sendAngles(const Angles& angles, int speed);
    bool sendAngle(int joint, double angle, int speed);  // joint 1..6
    void stop();
    std::optional<bool> isMoving();  // nullopt if no valid reply

    // ---- Feedback (nullopt if no valid reply after all retries) ------------
    std::optional<Angles> getAngles(int retries = 5);
    std::optional<Coords> getCoords(int retries = 5);

    // ---- Gripper and Atom LED ----------------------------------------------
    void setGripperState(int state, int speed);  // 0 = open, 1 = close
    void setColor(uint8_t r, uint8_t g, uint8_t b);

    // ---- Servo PID (configuration only: never call from a control loop) ---
    // Read one servo data value. Reading never changes the servo.
    // nullopt if the servo does not answer or the reply has an unknown format.
    std::optional<int> getServoData(int joint, uint8_t address, int retries = 3);

    // P, I and D of one joint (1..6); nullopt if any of them cannot be read.
    std::optional<Pid> getPid(int joint);

    // Write P, I and D of one joint and verify each value by reading it back.
    // Safeguards:
    //   * only the three PID addresses are ever written
    //   * throws std::invalid_argument if a value is negative, P or D is above
    //     PID_MAX_FACTOR x factory, or I is above PID_MAX_I
    //   * throws std::runtime_error, writing nothing, if the current values
    //     cannot be read first or the arm reports that it is moving
    // Returns true if every value was verified, false if any read-back failed.
    bool setPid(int joint, const Pid& pid);

    // Throws std::invalid_argument if `pid` is outside the safety bounds.
    static void checkPidBounds(int joint, const Pid& pid);

    // ---- Servo temperatures (read-only) -------------------------------------
    // Temperature of each joint's servo in deg C. Reading changes nothing.
    // nullopt if the arm does not answer or the reply has an unknown format.
    std::optional<ServoTemps> getTemps(int retries = 3);

    // Supply voltage of each joint's servo in V. Reading changes nothing.
    // nullopt if the arm does not answer or the reply has an unknown format.
    std::optional<ServoVolts> getVoltages(int retries = 3);

    // getTemps() plus the check against TEMP_WARM_C / TEMP_HOT_C.
    std::optional<TempCheck> checkTemps(int retries = 3);

    // Level of one temperature: Normal, Warm (>= TEMP_WARM_C), Hot (>= TEMP_HOT_C).
    static TempLevel tempLevel(int celsius);

private:
    int fd_ = -1;
    std::optional<Angles> lastAngles_;  // last angles read or commanded

    // Clamp one angle to JOINT_LIMITS of `joint` (1..6); sets `clamped`.
    static double limitAngle(int joint, double angle, bool& clamped);

    // Build a frame and write it to the port.
    void sendCommand(uint8_t cmd, const std::vector<uint8_t>& data = {});

    // Read until a complete frame for `cmd` arrives; return its data bytes.
    std::optional<std::vector<uint8_t>> readReply(uint8_t cmd, std::chrono::milliseconds timeout);

    // Send a request and wait for a reply with exactly `expected_size` data
    // bytes, retrying up to `retries` times.
    std::optional<std::vector<uint8_t>> query(uint8_t cmd, size_t expected_size, int retries = 5);

    // Send a request with data and return the reply's data bytes, any size.
    std::optional<std::vector<uint8_t>> queryWithData(uint8_t cmd, const std::vector<uint8_t>& data,
                                                      int retries);

    // Write one servo data byte, then read it back. True if it matches.
    bool writeServoByte(int joint, uint8_t address, int value);

    static void appendInt16(std::vector<uint8_t>& out, int16_t value);  // big-endian
    static int16_t readInt16(const std::vector<uint8_t>& data, size_t offset);
    static uint8_t clampSpeed(int speed);  // 1..100
};

}  // namespace mycobot
