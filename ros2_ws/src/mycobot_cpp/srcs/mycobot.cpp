// mycobot.cpp - implementation of mycobot::MyCobot280 (see mycobot.hpp).

#include "mycobot.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace mycobot {

using namespace std::chrono_literals;

namespace {

constexpr auto REPLY_TIMEOUT = 500ms;  // max wait for one reply
constexpr auto RETRY_DELAY = 100ms;    // pause between retries

speed_t toTermiosBaud(int baud) {
    switch (baud) {
        case 1000000:
            return B1000000;
        case 115200:
            return B115200;
        default:
            throw std::invalid_argument("unsupported baud rate: " + std::to_string(baud));
    }
}

std::runtime_error systemError(const std::string& what) {
    return std::runtime_error(what + ": " + std::strerror(errno));
}

// Forward kinematics for the height check. Joint frames from Elephant Robotics'
// official URDF of the Jetson Nano arm (mycobot_280_jn, the same origins as
// ros2_ws/src/braco_description, kept equal by its test_urdf.py): each joint is a
// fixed origin (xyz in m, rpy in rad, applied as Rz*Ry*Rx) followed by a rotation
// about its own z axis.
struct Transform {
    double r[3][3];
    double t[3];
};

Transform identity() { return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}}; }

Transform operator*(const Transform& a, const Transform& b) {
    Transform out{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) out.r[i][j] += a.r[i][k] * b.r[k][j];
        }
        out.t[i] = a.t[i];
        for (int k = 0; k < 3; ++k) out.t[i] += a.r[i][k] * b.t[k];
    }
    return out;
}

Transform jointTransform(const double xyz[3], const double rpy[3], double angle) {
    const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
    const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
    const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
    const double cq = std::cos(angle), sq = std::sin(angle);
    const Transform origin = {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                               {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                               {-sp, cp * sr, cp * cr}},
                              {xyz[0], xyz[1], xyz[2]}};
    const Transform spin = {{{cq, -sq, 0}, {sq, cq, 0}, {0, 0, 1}}, {0, 0, 0}};
    return origin * spin;
}

struct JointOrigin {
    double xyz[3];
    double rpy[3];
};

constexpr double HALF_PI = 1.5707963267948966;
constexpr JointOrigin JOINT_ORIGINS[6] = {
    {{0, 0, 0.15756}, {0, 0, 0}},                // J1 (from the bottom of the casing)
    {{0, 0, -0.001}, {0, HALF_PI, -HALF_PI}},    // J2
    {{-0.1104, 0, 0}, {0, 0, 0}},                // J3
    {{-0.096, 0, 0.06462}, {0, 0, -HALF_PI}},    // J4
    {{0, -0.07318, 0}, {HALF_PI, -HALF_PI, 0}},  // J5
    {{0, 0.0456, 0}, {-HALF_PI, 0, 0}},          // J6 (flange)
};

}  // namespace

double minHeightMm(const Angles& angles) {
    constexpr double DEG = 3.14159265358979323846 / 180.0;
    // The chain starts at the bottom of the Jetson casing.
    constexpr double ORIGIN_MM = BASE_HEIGHT_MM - CASING_HEIGHT_MM;
    Transform pose = identity();
    double lowest = 1e9;
    for (int i = 0; i < 6; ++i) {
        pose = pose * jointTransform(JOINT_ORIGINS[i].xyz, JOINT_ORIGINS[i].rpy, angles[i] * DEG);
        if (i >= 3) lowest = std::min(lowest, pose.t[2] * 1000.0 + ORIGIN_MM);  // J4, J5, J6
    }
    // The tool points along the flange z axis.
    const double tip_z = pose.t[2] + pose.r[2][2] * TOOL_LENGTH_MM / 1000.0;
    return std::min(lowest, tip_z * 1000.0 + ORIGIN_MM);
}

std::optional<Angles> stepToward(const Angles& current, const Angles& pose, double lead_deg,
                                 double tolerance_deg) {
    double deviation = 0.0;
    for (size_t i = 0; i < pose.size(); ++i) {
        deviation = std::max(deviation, std::abs(current[i] - pose[i]));
    }
    if (deviation <= tolerance_deg) return std::nullopt;

    const double scale = std::min(1.0, lead_deg / deviation);
    Angles target{};
    for (size_t i = 0; i < target.size(); ++i) {
        target[i] = std::round((current[i] + (pose[i] - current[i]) * scale) * 10.0) / 10.0;
    }
    return target;
}

// ---------------------------------------------------------------- lifecycle

MyCobot280::MyCobot280(const std::string& port, int baud) {
    speed_t termios_baud = toTermiosBaud(baud);

    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) { throw systemError("could not open " + port); }

    // Exclusive access: a second (non-root) opener, e.g. an old script run on the host while
    // arm_driver is up, now gets EBUSY at once instead of interleaving bytes with the driver.
    // Not fatal if the ioctl is unsupported.
    ioctl(fd_, TIOCEXCL);

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close(fd_);
        throw systemError("tcgetattr failed on " + port);
    }
    cfmakeraw(&tty);  // 8N1, no echo, no line processing
    cfsetispeed(&tty, termios_baud);
    cfsetospeed(&tty, termios_baud);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;   // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;  // no hardware flow control
    tty.c_cc[VMIN] = 0;       // read() returns after the timeout...
    tty.c_cc[VTIME] = 1;      // ...of 100 ms if nothing arrives
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close(fd_);
        throw systemError("tcsetattr failed on " + port);
    }
    tcflush(fd_, TCIOFLUSH);
}

MyCobot280::~MyCobot280() {
    if (fd_ >= 0) { close(fd_); }
}

// -------------------------------------------------------------------- power

void MyCobot280::powerOn() { sendCommand(protocol::POWER_ON); }

void MyCobot280::powerOff() { sendCommand(protocol::POWER_OFF); }

void MyCobot280::releaseAllServos() { sendCommand(protocol::RELEASE_ALL_SERVOS); }

std::optional<bool> MyCobot280::isPowerOn() {
    auto data = query(protocol::IS_POWER_ON, 1);
    if (!data) return std::nullopt;
    return (*data)[0] == 1;
}

// ------------------------------------------------------------ joint limits

double MyCobot280::limitAngle(int joint, double angle, bool& clamped) {
    if (!std::isfinite(angle)) {
        throw std::invalid_argument("J" + std::to_string(joint) + ": angle is not a finite number");
    }
    const JointLimit& lim = JOINT_LIMITS[joint - 1];
    const double limited = std::clamp(angle, lim.min, lim.max);
    if (limited != angle) clamped = true;
    return limited;
}

// ------------------------------------------------------------------- motion

// A move is allowed if its lowest point stays above the margin, or at least
// does not get lower than the arm already is (so a pose that is already too
// low can still be left).
static bool heightAllowed(const Angles& target, const std::optional<Angles>& current) {
    const double height = minHeightMm(target);
    if (height >= TABLE_MARGIN_MM) return true;
    return current && height >= minHeightMm(*current);
}

bool MyCobot280::sendAngles(const Angles& angles, int speed) {
    bool clamped = false;
    Angles limited{};
    for (size_t i = 0; i < angles.size(); ++i) {
        limited[i] = limitAngle(static_cast<int>(i) + 1, angles[i], clamped);
    }
    if (!heightAllowed(limited, lastAngles_)) return true;

    std::vector<uint8_t> data;
    for (double a : limited) appendInt16(data, static_cast<int16_t>(std::lround(a * 100.0)));
    data.push_back(clampSpeed(speed));
    sendCommand(protocol::SEND_ANGLES, data);
    lastAngles_ = limited;
    return clamped;
}

bool MyCobot280::sendAngle(int joint, double angle, int speed) {
    if (joint < 1 || joint > 6) {
        throw std::invalid_argument("joint must be 1..6, got " + std::to_string(joint));
    }
    bool clamped = false;
    const double a = limitAngle(joint, angle, clamped);

    if (!lastAngles_) lastAngles_ = getAngles();
    if (!lastAngles_) return true;  // pose unknown: cannot prove the move is safe
    Angles target = *lastAngles_;
    target[joint - 1] = a;
    if (!heightAllowed(target, lastAngles_)) return true;

    std::vector<uint8_t> data;
    data.push_back(static_cast<uint8_t>(joint));
    appendInt16(data, static_cast<int16_t>(std::lround(a * 100.0)));
    data.push_back(clampSpeed(speed));
    sendCommand(protocol::SEND_ANGLE, data);
    lastAngles_ = target;
    return clamped;
}

void MyCobot280::stop() { sendCommand(protocol::STOP); }

std::optional<bool> MyCobot280::isMoving() {
    auto data = query(protocol::IS_MOVING, 1);
    if (!data) return std::nullopt;
    return (*data)[0] == 1;
}

// ----------------------------------------------------------------- feedback

std::optional<Angles> MyCobot280::getAngles(int retries) {
    auto data = query(protocol::GET_ANGLES, 12, retries);
    if (!data) return std::nullopt;

    Angles angles{};
    for (size_t i = 0; i < 6; ++i) { angles[i] = readInt16(*data, 2 * i) / 100.0; }
    lastAngles_ = angles;
    return angles;
}

std::optional<Coords> MyCobot280::getCoords(int retries) {
    auto data = query(protocol::GET_COORDS, 12, retries);
    if (!data) return std::nullopt;

    Coords coords{};
    for (size_t i = 0; i < 3; ++i) {
        coords[i] = readInt16(*data, 2 * i) / 10.0;  // X, Y, Z (mm)
    }
    for (size_t i = 3; i < 6; ++i) {
        coords[i] = readInt16(*data, 2 * i) / 100.0;  // RX, RY, RZ (deg)
    }
    return coords;
}

// --------------------------------------------------------- gripper and LED

void MyCobot280::setGripperState(int state, int speed) {
    std::vector<uint8_t> data = {static_cast<uint8_t>(state ? 1 : 0), clampSpeed(speed)};
    sendCommand(protocol::SET_GRIPPER_STATE, data);
}

void MyCobot280::setColor(uint8_t r, uint8_t g, uint8_t b) {
    sendCommand(protocol::SET_COLOR, {r, g, b});
}

// ---------------------------------------------------------------- servo PID

namespace {

constexpr auto SERVO_WRITE_SETTLE = 200ms;  // pause before reading a written value back
constexpr int SERVO_WRITE_ATTEMPTS = 3;

void checkJoint(int joint) {
    if (joint < 1 || joint > 6) {
        throw std::invalid_argument("joint must be 1..6, got " + std::to_string(joint));
    }
}

}  // namespace

std::optional<int> MyCobot280::getServoData(int joint, uint8_t address, int retries) {
    checkJoint(joint);
    const uint8_t id = static_cast<uint8_t>(joint);
    auto data = queryWithData(protocol::GET_SERVO_DATA, {id, address}, retries);
    if (!data) return std::nullopt;

    // Reply formats seen across firmware versions; anything else is refused.
    switch (data->size()) {
        case 1:  // value
            return (*data)[0];
        case 2:  // 16-bit value, high byte first
            return (static_cast<int>((*data)[0]) << 8) | (*data)[1];
        case 3:  // joint, address, value (echo of the request)
            if ((*data)[0] == id && (*data)[1] == address) return (*data)[2];
            return std::nullopt;
        default:
            return std::nullopt;
    }
}

std::optional<Pid> MyCobot280::getPid(int joint) {
    const auto p = getServoData(joint, servo::ADDR_P);
    const auto i = getServoData(joint, servo::ADDR_I);
    const auto d = getServoData(joint, servo::ADDR_D);
    if (!p || !i || !d) return std::nullopt;
    return Pid{*p, *i, *d};
}

void MyCobot280::checkPidBounds(int joint, const Pid& pid) {
    checkJoint(joint);
    const Pid& factory = FACTORY_PID[joint - 1];
    const int maxP = static_cast<int>(factory.p * PID_MAX_FACTOR);
    const int maxD = static_cast<int>(factory.d * PID_MAX_FACTOR);
    const std::string j = "J" + std::to_string(joint);

    if (pid.p < 0 || pid.i < 0 || pid.d < 0) {
        throw std::invalid_argument(j + ": PID values cannot be negative");
    }
    if (pid.p > maxP) {
        throw std::invalid_argument(j + ": P " + std::to_string(pid.p) + " above safe maximum " +
                                    std::to_string(maxP));
    }
    if (pid.i > PID_MAX_I) {
        throw std::invalid_argument(j + ": I " + std::to_string(pid.i) + " above safe maximum " +
                                    std::to_string(PID_MAX_I));
    }
    if (pid.d > maxD) {
        throw std::invalid_argument(j + ": D " + std::to_string(pid.d) + " above safe maximum " +
                                    std::to_string(maxD));
    }
}

bool MyCobot280::setPid(int joint, const Pid& pid) {
    checkPidBounds(joint, pid);  // throws before anything is sent

    // Only write once reading works: proves the protocol before changing anything.
    const auto current = getPid(joint);
    if (!current) {
        throw std::runtime_error("J" + std::to_string(joint) +
                                 ": cannot read current PID values, nothing was written");
    }
    if (isMoving().value_or(false)) {
        throw std::runtime_error("arm is moving, PID values were not changed");
    }

    bool ok = true;
    if (current->p != pid.p) ok = writeServoByte(joint, servo::ADDR_P, pid.p) && ok;
    if (current->i != pid.i) ok = writeServoByte(joint, servo::ADDR_I, pid.i) && ok;
    if (current->d != pid.d) ok = writeServoByte(joint, servo::ADDR_D, pid.d) && ok;
    return ok;
}

bool MyCobot280::writeServoByte(int joint, uint8_t address, int value) {
    // Private: only called by setPid() with a PID address and a checked value.
    if (address != servo::ADDR_P && address != servo::ADDR_I && address != servo::ADDR_D) {
        throw std::logic_error("writeServoByte: address " + std::to_string(address) +
                               " is not a PID address");
    }
    const std::vector<uint8_t> data = {static_cast<uint8_t>(joint), address,
                                       static_cast<uint8_t>(value)};
    for (int attempt = 0; attempt < SERVO_WRITE_ATTEMPTS; ++attempt) {
        sendCommand(protocol::SET_SERVO_DATA, data);
        std::this_thread::sleep_for(SERVO_WRITE_SETTLE);
        if (getServoData(joint, address) == value) return true;
    }
    return false;
}

// ------------------------------------------------------------- servo status

std::optional<ServoTemps> MyCobot280::getTemps(int retries) {
    auto data = queryWithData(protocol::GET_SERVO_TEMPS, {}, retries);
    if (!data) return std::nullopt;

    ServoTemps temps{};
    if (data->size() == 6) {  // one byte per joint
        for (size_t i = 0; i < 6; ++i) temps[i] = (*data)[i];
    } else if (data->size() == 12) {  // 16-bit value per joint, high byte first
        for (size_t i = 0; i < 6; ++i) temps[i] = readInt16(*data, 2 * i);
    } else {
        return std::nullopt;  // unknown format: refuse to guess
    }
    return temps;
}

std::optional<ServoVolts> MyCobot280::getVoltages(int retries) {
    auto data = queryWithData(protocol::GET_SERVO_VOLTAGES, {}, retries);
    if (!data || data->size() != 6) return std::nullopt;  // one byte per joint, else do not guess

    ServoVolts volts{};
    for (size_t i = 0; i < 6; ++i) volts[i] = (*data)[i] * VOLTAGE_STEP_V;
    return volts;
}

TempLevel MyCobot280::tempLevel(int celsius) {
    if (celsius >= TEMP_HOT_C) return TempLevel::Hot;
    if (celsius >= TEMP_WARM_C) return TempLevel::Warm;
    return TempLevel::Normal;
}

TempCheck evaluateTemps(const ServoTemps& temps) {
    int hottest = 0;  // index of the hottest joint
    for (int i = 1; i < 6; ++i) {
        if (temps[i] > temps[hottest]) hottest = i;
    }
    return TempCheck{temps, MyCobot280::tempLevel(temps[hottest]), hottest + 1};
}

Rgb tempColour(TempLevel level) {
    switch (level) {
        case TempLevel::Warm:
            return {255, 255, 0};
        case TempLevel::Hot:
            return {255, 0, 0};
        default:
            return {0, 255, 0};
    }
}

std::optional<TempCheck> MyCobot280::checkTemps(int retries) {
    const auto temps = getTemps(retries);
    if (!temps) return std::nullopt;
    return evaluateTemps(*temps);
}

// ------------------------------------------------------------ serial layer

void MyCobot280::sendCommand(uint8_t cmd, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> frame = {protocol::HEADER, protocol::HEADER,
                                  static_cast<uint8_t>(data.size() + 2), cmd};
    frame.insert(frame.end(), data.begin(), data.end());
    frame.push_back(protocol::FOOTER);

    size_t sent = 0;
    while (sent < frame.size()) {
        ssize_t n = write(fd_, frame.data() + sent, frame.size() - sent);
        if (n < 0) {
            if (errno == EINTR) continue;
            throw systemError("serial write failed");
        }
        sent += static_cast<size_t>(n);
    }
    tcdrain(fd_);  // wait until the bytes have left the UART
}

std::optional<std::vector<uint8_t>> MyCobot280::readReply(uint8_t cmd,
                                                          std::chrono::milliseconds timeout) {
    std::vector<uint8_t> buf;
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t tmp[64];
        ssize_t n = read(fd_, tmp, sizeof(tmp));
        if (n < 0 && errno != EINTR && errno != EAGAIN) { throw systemError("serial read failed"); }
        if (n > 0) { buf.insert(buf.end(), tmp, tmp + n); }

        // Look for: HEADER HEADER len cmd data... FOOTER
        for (size_t i = 0; i + 4 < buf.size(); ++i) {
            if (buf[i] != protocol::HEADER || buf[i + 1] != protocol::HEADER) continue;
            const size_t len = buf[i + 2];
            const size_t footer = i + 2 + len;
            if (len < 2 || footer >= buf.size()) continue;  // incomplete or invalid
            if (buf[i + 3] == cmd && buf[footer] == protocol::FOOTER) {
                return std::vector<uint8_t>(buf.begin() + static_cast<std::ptrdiff_t>(i + 4),
                                            buf.begin() + static_cast<std::ptrdiff_t>(footer));
            }
        }
    }
    return std::nullopt;
}

std::optional<std::vector<uint8_t>> MyCobot280::query(uint8_t cmd, size_t expected_size,
                                                      int retries) {
    for (int attempt = 0; attempt < std::max(1, retries); ++attempt) {
        tcflush(fd_, TCIFLUSH);  // drop stale bytes from earlier commands
        sendCommand(cmd);
        auto data = readReply(cmd, REPLY_TIMEOUT);
        if (data && data->size() == expected_size) { return data; }
        std::this_thread::sleep_for(RETRY_DELAY);
    }
    return std::nullopt;
}

std::optional<std::vector<uint8_t>> MyCobot280::queryWithData(uint8_t cmd,
                                                              const std::vector<uint8_t>& data,
                                                              int retries) {
    for (int attempt = 0; attempt < std::max(1, retries); ++attempt) {
        tcflush(fd_, TCIFLUSH);
        sendCommand(cmd, data);
        auto reply = readReply(cmd, REPLY_TIMEOUT);
        if (reply && !reply->empty()) { return reply; }
        std::this_thread::sleep_for(RETRY_DELAY);
    }
    return std::nullopt;
}

// ------------------------------------------------------------------ helpers

std::string envOr(const char* name, const std::string& fallback) {
    const char* value = std::getenv(name);
    return value && *value ? value : fallback;
}

void MyCobot280::appendInt16(std::vector<uint8_t>& out, int16_t value) {
    const uint16_t v = static_cast<uint16_t>(value);
    out.push_back(static_cast<uint8_t>(v >> 8));  // high byte first
    out.push_back(static_cast<uint8_t>(v & 0xFF));
}

int16_t MyCobot280::readInt16(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<int16_t>((data[offset] << 8) | data[offset + 1]);
}

uint8_t MyCobot280::clampSpeed(int speed) {
    return static_cast<uint8_t>(std::clamp(speed, 1, 100));
}

}  // namespace mycobot
