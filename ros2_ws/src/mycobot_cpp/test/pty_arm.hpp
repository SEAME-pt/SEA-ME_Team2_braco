// pty_arm.hpp - a fake myCobot 280 on a pseudo-terminal, for tests. Header-only, test code: it is
// installed with the package (other packages' tests use it) but is not part of the library, and it
// lives in test/ so coverage does not measure it. Link `util` for openpty().
//
// The test holds the master side; MyCobot280, or a program started by the test, opens path().
// Two ways to use it, not both at once:
//   manual   sent() returns the bytes written to the port, respond() answers the next requests
//            with a fixed reply.
//   serve()  a thread answers like the arm, from state(): angles, temperatures, voltages, power,
//            moving and servo data. SEND_ANGLES / SEND_ANGLE move the angles there at once and
//            SET_SERVO_DATA stores the value (unless ignoreWrites). Every frame is kept in received().

#pragma once

#include <poll.h>
#include <pty.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "mycobot.hpp"

namespace mycobot::fake {

using Bytes = std::vector<uint8_t>;

// Big-endian int16, as the arm sends it.
inline void appendInt16(Bytes& out, int value) {
    const uint16_t v = static_cast<uint16_t>(static_cast<int16_t>(value));
    out.push_back(static_cast<uint8_t>(v >> 8));
    out.push_back(static_cast<uint8_t>(v & 0xFF));
}

inline int readInt16(const Bytes& data, size_t offset) {
    return static_cast<int16_t>((data[offset] << 8) | data[offset + 1]);
}

// HEADER HEADER len cmd data... FOOTER
inline Bytes frame(uint8_t cmd, const Bytes& data = {}) {
    Bytes f = {protocol::HEADER, protocol::HEADER, static_cast<uint8_t>(data.size() + 2), cmd};
    f.insert(f.end(), data.begin(), data.end());
    f.push_back(protocol::FOOTER);
    return f;
}

struct Frame {
    uint8_t cmd;
    Bytes data;
};

class PtyArm {
public:
    // What serve() answers with.
    struct State {
        Angles angles = ZERO_ANGLES;
        ServoTemps temps = {35, 35, 35, 35, 35, 35};
        std::array<int, 6> decivolts = {120, 120, 120, 120, 120, 120};  // 0.1 V steps
        bool powered = true;
        bool moving = false;
        std::map<std::pair<int, uint8_t>, int> servoData;  // (joint, address) -> value
        bool silent = false;                               // true: receive, never answer
        bool ignoreWrites = false;   // true: SET_SERVO_DATA is received but not stored
        std::set<int> brokenServos;  // their servo data comes back in an unknown format: no value,
                                     // at once (a silent arm costs 600 ms per read)

        State() {
            for (int j = 1; j <= 6; ++j) {
                servoData[{j, servo::ADDR_P}] = FACTORY_PID[j - 1].p;
                servoData[{j, servo::ADDR_I}] = FACTORY_PID[j - 1].i;
                servoData[{j, servo::ADDR_D}] = FACTORY_PID[j - 1].d;
            }
        }
    };

    PtyArm() {
        char name[128];
        if (openpty(&master_, &slave_, name, nullptr, nullptr) != 0) {
            throw std::runtime_error("openpty failed");
        }
        path_ = name;
    }

    ~PtyArm() {
        stop();
        close(slave_);
        close(master_);
    }

    PtyArm(const PtyArm&) = delete;
    PtyArm& operator=(const PtyArm&) = delete;

    const std::string& path() const { return path_; }

    // MyCobot280 opens the port in exclusive mode (TIOCEXCL), and the mode outlives it here: this
    // object keeps the port open. Clear it before another program opens the port (a non-root
    // opener would get EBUSY).
    void unlock() { ioctl(slave_, TIOCNXCL); }

    // ---- manual ---------------------------------------------------------------------

    // Every byte written to the port, waiting up to `timeout` for the first one.
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

    // Answer the next `count` requests with `reply`, from another thread (the class blocks in its
    // read). wait() joins it.
    void respond(const Bytes& reply, int count = 1) {
        wait();
        worker_ = std::thread([this, reply, count] {
            for (int i = 0; i < count; ++i) {
                if (sent(std::chrono::milliseconds(2000)).empty()) return;
                if (write(master_, reply.data(), reply.size()) !=
                    static_cast<ssize_t>(reply.size())) {
                    return;  // the test sees the missing reply
                }
            }
        });
    }

    void wait() {
        if (worker_.joinable()) worker_.join();
    }

    // ---- serve ----------------------------------------------------------------------

    void serve() {
        stop();
        quit_ = false;
        worker_ = std::thread([this] { serveLoop(); });
    }

    // Stops serve() (or waits for respond()).
    void stop() {
        quit_ = true;
        wait();
    }

    State state() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

    // Change the state while serving: arm.update([](auto& s) { s.temps[2] = 70; });
    template <typename F>
    void update(F change) {
        std::lock_guard<std::mutex> lock(mutex_);
        change(state_);
    }

    std::vector<Frame> received() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return received_;
    }

    std::vector<Frame> received(uint8_t cmd) const {
        std::vector<Frame> out;
        for (const auto& f : received()) {
            if (f.cmd == cmd) out.push_back(f);
        }
        return out;
    }

    // Wait up to `timeout` until a frame with `cmd` has been received.
    bool waitFor(uint8_t cmd, std::chrono::milliseconds timeout) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (!received(cmd).empty()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return !received(cmd).empty();
    }

private:
    int master_ = -1;
    int slave_ = -1;  // kept open, so the port stays up between two programs opening it
    std::string path_;
    std::thread worker_;
    std::atomic<bool> quit_{false};
    mutable std::mutex mutex_;
    State state_;
    std::vector<Frame> received_;

    void serveLoop() {
        Bytes buf;
        pollfd p{master_, POLLIN, 0};
        while (!quit_) {
            if (poll(&p, 1, 50) <= 0) continue;
            uint8_t tmp[256];
            const ssize_t n = read(master_, tmp, sizeof(tmp));
            if (n <= 0) continue;
            buf.insert(buf.end(), tmp, tmp + n);

            // Take every complete frame off the front; drop bytes that cannot start one.
            while (buf.size() >= 5) {
                if (buf[0] != protocol::HEADER || buf[1] != protocol::HEADER) {
                    buf.erase(buf.begin());
                    continue;
                }
                const size_t end = 2 + static_cast<size_t>(buf[2]);  // index of the footer
                if (end >= buf.size()) break;
                Frame f{buf[3], Bytes(buf.begin() + 4, buf.begin() + static_cast<long>(end))};
                buf.erase(buf.begin(), buf.begin() + static_cast<long>(end) + 1);
                const Bytes reply = handle(f);
                if (!reply.empty() && write(master_, reply.data(), reply.size()) < 0) return;
            }
        }
    }

    // The reply to one frame (empty: none), and its effect on the state.
    Bytes handle(const Frame& f) {
        std::lock_guard<std::mutex> lock(mutex_);
        received_.push_back(f);
        State& s = state_;
        Bytes data;
        switch (f.cmd) {
            case protocol::GET_ANGLES:
                for (double a : s.angles) appendInt16(data, static_cast<int>(std::lround(a * 100)));
                break;
            case protocol::GET_COORDS:
                data.assign(12, 0);
                break;
            case protocol::GET_SERVO_TEMPS:
                for (int t : s.temps) data.push_back(static_cast<uint8_t>(t));
                break;
            case protocol::GET_SERVO_VOLTAGES:
                for (int v : s.decivolts) data.push_back(static_cast<uint8_t>(v));
                break;
            case protocol::IS_POWER_ON:
                data = {static_cast<uint8_t>(s.powered)};
                break;
            case protocol::IS_MOVING:
                data = {static_cast<uint8_t>(s.moving)};
                break;
            case protocol::GET_SERVO_DATA:  // joint, address -> echo + value
                if (f.data.size() != 2) return {};
                if (s.brokenServos.count(f.data[0])) {
                    data = {0, 0, 0, 0};
                    break;
                }
                data = {f.data[0], f.data[1],
                        static_cast<uint8_t>(s.servoData[{f.data[0], f.data[1]}])};
                break;
            case protocol::SET_SERVO_DATA:  // joint, address, value
                if (f.data.size() == 3 && !s.ignoreWrites)
                    s.servoData[{f.data[0], f.data[1]}] = f.data[2];
                return {};
            case protocol::SEND_ANGLES:  // 6 x int16 hundredths, speed
                if (f.data.size() == 13) {
                    for (size_t i = 0; i < 6; ++i) s.angles[i] = readInt16(f.data, 2 * i) / 100.0;
                }
                return {};
            case protocol::SEND_ANGLE:  // joint, int16 hundredths, speed
                if (f.data.size() == 4 && f.data[0] >= 1 && f.data[0] <= 6) {
                    s.angles[f.data[0] - 1] = readInt16(f.data, 1) / 100.0;
                }
                return {};
            case protocol::POWER_ON:
                s.powered = true;
                return {};
            case protocol::POWER_OFF:
                s.powered = false;
                return {};
            default:
                return {};
        }
        if (s.silent) return {};
        return frame(f.cmd, data);
    }
};

}  // namespace mycobot::fake
