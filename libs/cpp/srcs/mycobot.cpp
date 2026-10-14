// mycobot.cpp - implementation of mycobot::MyCobot280 (see mycobot.hpp).

#include "mycobot.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>

namespace mycobot {

using namespace std::chrono_literals;

namespace {

constexpr auto REPLY_TIMEOUT = 500ms;   // max wait for one reply
constexpr auto RETRY_DELAY = 100ms;     // pause between retries

speed_t toTermiosBaud(int baud) {
    switch (baud) {
        case 1000000: return B1000000;
        case 115200:  return B115200;
        default:
            throw std::invalid_argument("unsupported baud rate: " + std::to_string(baud));
    }
}

std::runtime_error systemError(const std::string& what) {
    return std::runtime_error(what + ": " + std::strerror(errno));
}

}  // namespace

// ---------------------------------------------------------------- lifecycle

MyCobot280::MyCobot280(const std::string& port, int baud) {
    speed_t termios_baud = toTermiosBaud(baud);

    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        throw systemError("could not open " + port);
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close(fd_);
        throw systemError("tcgetattr failed on " + port);
    }
    cfmakeraw(&tty);                    // 8N1, no echo, no line processing
    cfsetispeed(&tty, termios_baud);
    cfsetospeed(&tty, termios_baud);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;            // no hardware flow control
    tty.c_cc[VMIN] = 0;                 // read() returns after the timeout...
    tty.c_cc[VTIME] = 1;                // ...of 100 ms if nothing arrives
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close(fd_);
        throw systemError("tcsetattr failed on " + port);
    }
    tcflush(fd_, TCIOFLUSH);
}

MyCobot280::~MyCobot280() {
    if (fd_ >= 0) {
        close(fd_);
    }
}

// -------------------------------------------------------------------- power

void MyCobot280::powerOn() {
    sendCommand(protocol::POWER_ON);
}

void MyCobot280::powerOff() {
    sendCommand(protocol::POWER_OFF);
}

void MyCobot280::releaseAllServos() {
    sendCommand(protocol::RELEASE_ALL_SERVOS);
}

std::optional<bool> MyCobot280::isPowerOn() {
    auto data = query(protocol::IS_POWER_ON, 1);
    if (!data) return std::nullopt;
    return (*data)[0] == 1;
}

// ------------------------------------------------------------ joint limits

void MyCobot280::setSoftLimits(const JointLimits& limits) {
    for (size_t i = 0; i < limits.size(); ++i) {
        const JointLimit& soft = limits[i];
        const JointLimit& hard = FIRMWARE_LIMITS[i];
        const std::string joint = "J" + std::to_string(i + 1);
        if (!std::isfinite(soft.min) || !std::isfinite(soft.max) || soft.min >= soft.max) {
            throw std::invalid_argument(joint + " soft limit: min must be below max");
        }
        if (soft.min < hard.min || soft.max > hard.max) {
            throw std::invalid_argument(joint + " soft limit outside firmware range [" +
                                        std::to_string(hard.min) + ", " +
                                        std::to_string(hard.max) + "]");
        }
    }
    softLimits_ = limits;
}

double MyCobot280::limitAngle(int joint, double angle, bool& clamped) const {
    if (!std::isfinite(angle)) {
        throw std::invalid_argument("J" + std::to_string(joint) + ": angle is not a finite number");
    }
    const JointLimit& lim = softLimits_[joint - 1];
    const double limited = std::clamp(angle, lim.min, lim.max);
    if (limited != angle) clamped = true;
    return limited;
}

// ------------------------------------------------------------------- motion

bool MyCobot280::sendAngles(const Angles& angles, int speed) {
    bool clamped = false;
    std::vector<uint8_t> data;
    for (size_t i = 0; i < angles.size(); ++i) {
        const double a = limitAngle(static_cast<int>(i) + 1, angles[i], clamped);
        appendInt16(data, static_cast<int16_t>(std::lround(a * 100.0)));
    }
    data.push_back(clampSpeed(speed));
    sendCommand(protocol::SEND_ANGLES, data);
    return clamped;
}

bool MyCobot280::sendAngle(int joint, double angle, int speed) {
    if (joint < 1 || joint > 6) {
        throw std::invalid_argument("joint must be 1..6, got " + std::to_string(joint));
    }
    bool clamped = false;
    const double a = limitAngle(joint, angle, clamped);
    std::vector<uint8_t> data;
    data.push_back(static_cast<uint8_t>(joint));
    appendInt16(data, static_cast<int16_t>(std::lround(a * 100.0)));
    data.push_back(clampSpeed(speed));
    sendCommand(protocol::SEND_ANGLE, data);
    return clamped;
}

void MyCobot280::stop() {
    sendCommand(protocol::STOP);
}

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
    for (size_t i = 0; i < 6; ++i) {
        angles[i] = readInt16(*data, 2 * i) / 100.0;
    }
    return angles;
}

std::optional<Coords> MyCobot280::getCoords(int retries) {
    auto data = query(protocol::GET_COORDS, 12, retries);
    if (!data) return std::nullopt;

    Coords coords{};
    for (size_t i = 0; i < 3; ++i) {
        coords[i] = readInt16(*data, 2 * i) / 10.0;        // X, Y, Z (mm)
    }
    for (size_t i = 3; i < 6; ++i) {
        coords[i] = readInt16(*data, 2 * i) / 100.0;       // RX, RY, RZ (deg)
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
    tcdrain(fd_);   // wait until the bytes have left the UART
}

std::optional<std::vector<uint8_t>> MyCobot280::readReply(
        uint8_t cmd, std::chrono::milliseconds timeout) {
    std::vector<uint8_t> buf;
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline) {
        uint8_t tmp[64];
        ssize_t n = read(fd_, tmp, sizeof(tmp));
        if (n < 0 && errno != EINTR && errno != EAGAIN) {
            throw systemError("serial read failed");
        }
        if (n > 0) {
            buf.insert(buf.end(), tmp, tmp + n);
        }

        // Look for: HEADER HEADER len cmd data... FOOTER
        for (size_t i = 0; i + 4 < buf.size(); ++i) {
            if (buf[i] != protocol::HEADER || buf[i + 1] != protocol::HEADER) continue;
            const size_t len = buf[i + 2];
            const size_t footer = i + 2 + len;
            if (len < 2 || footer >= buf.size()) continue;   // incomplete or invalid
            if (buf[i + 3] == cmd && buf[footer] == protocol::FOOTER) {
                return std::vector<uint8_t>(buf.begin() + i + 4, buf.begin() + footer);
            }
        }
    }
    return std::nullopt;
}

std::optional<std::vector<uint8_t>> MyCobot280::query(
        uint8_t cmd, size_t expected_size, int retries) {
    for (int attempt = 0; attempt < std::max(1, retries); ++attempt) {
        tcflush(fd_, TCIFLUSH);   // drop stale bytes from earlier commands
        sendCommand(cmd);
        auto data = readReply(cmd, REPLY_TIMEOUT);
        if (data && data->size() == expected_size) {
            return data;
        }
        std::this_thread::sleep_for(RETRY_DELAY);
    }
    return std::nullopt;
}

// ------------------------------------------------------------------ helpers

void MyCobot280::appendInt16(std::vector<uint8_t>& out, int16_t value) {
    const uint16_t v = static_cast<uint16_t>(value);
    out.push_back(static_cast<uint8_t>(v >> 8));     // high byte first
    out.push_back(static_cast<uint8_t>(v & 0xFF));
}

int16_t MyCobot280::readInt16(const std::vector<uint8_t>& data, size_t offset) {
    return static_cast<int16_t>((data[offset] << 8) | data[offset + 1]);
}

uint8_t MyCobot280::clampSpeed(int speed) {
    return static_cast<uint8_t>(std::clamp(speed, 1, 100));
}

}  // namespace mycobot
