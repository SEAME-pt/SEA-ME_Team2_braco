// test_tools.cpp - GoogleTests for the stand-alone programs (srcs/park.cpp, zero.cpp, ...), run as
// the built binaries (TOOLS_DIR, from CMake) with their devices pointed at fakes through MYCOBOT_PORT,
// MYCOBOT_JOYSTICK and MYCOBOT_LCD_BUS. The arm is a PtyArm (pty_arm.hpp) answering like the real
// one, the joystick a FIFO. A program that returns from main() writes its coverage data like any
// other process, so the loops are ended with SIGINT (Ctrl+C), which every tool handles.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <linux/joystick.h>
#include <poll.h>
#include <pty.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "mycobot.hpp"
#include "pty_arm.hpp"

namespace mc = mycobot;
using namespace std::chrono_literals;

namespace {

constexpr const char* MISSING = "/dev/does-not-exist";

struct Result {
    int exitCode = -1;  // -1: killed after the timeout
    std::string out;
    std::string err;
};

struct Options {
    std::map<std::string, std::string> env;  // added to the environment
    std::string input;                       // stdin, then end of input
    bool terminal = false;                   // stdout on a pseudo-terminal instead of a pipe
};

// One run of TOOLS_DIR/<tool>, started by the constructor. A thread collects its output, so the
// test can wait for a line while the program runs.
class Tool {
public:
    Tool(const std::string& tool, const std::vector<std::string>& args,
         const Options& options = {}) {
        int out[2] = {-1, -1};
        int err[2] = {-1, -1};
        int in[2] = {-1, -1};
        if (options.terminal) {
            if (openpty(&out[0], &out[1], nullptr, nullptr, nullptr) != 0) return;
        } else if (pipe(out) != 0) {
            return;
        }
        if (pipe(err) != 0 || pipe(in) != 0) return;
        const std::string path = std::string(TOOLS_DIR) + "/" + tool;

        pid_ = fork();
        if (pid_ == 0) {
            dup2(in[0], STDIN_FILENO);
            dup2(out[1], STDOUT_FILENO);
            dup2(err[1], STDERR_FILENO);
            for (int fd : {in[0], in[1], out[0], out[1], err[0], err[1]}) close(fd);
            for (const auto& [name, value] : options.env) setenv(name.c_str(), value.c_str(), 1);
            std::vector<char*> argv = {const_cast<char*>(path.c_str())};
            for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
            argv.push_back(nullptr);
            execv(path.c_str(), argv.data());
            _exit(127);
        }
        close(in[0]);
        close(out[1]);
        close(err[1]);
        if (!options.input.empty()) {
            [[maybe_unused]] const ssize_t n =
                write(in[1], options.input.data(), options.input.size());
        }
        close(in[1]);
        reader_ = std::thread([this, fds = std::array<int, 2>{out[0], err[0]}] { collect(fds); });
    }

    ~Tool() {
        if (pid_ > 0 && !exited_) wait(0ms);
        if (reader_.joinable()) reader_.join();
    }

    Tool(const Tool&) = delete;
    Tool& operator=(const Tool&) = delete;

    // Wait until `text` appears in stdout after the previous match; true if it did.
    bool next(const std::string& text, std::chrono::milliseconds timeout = 10s) {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (true) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                const size_t at = out_.find(text, cursor_);
                if (at != std::string::npos) {
                    cursor_ = at + text.size();
                    return true;
                }
            }
            if (std::chrono::steady_clock::now() > deadline) return false;
            std::this_thread::sleep_for(20ms);
        }
    }

    void interrupt() { kill(pid_, SIGINT); }

    // Wait for the exit; kill the program after `timeout`.
    Result wait(std::chrono::milliseconds timeout = 20s) {
        Result result;
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        int status = 0;
        while (waitpid(pid_, &status, WNOHANG) == 0) {
            if (std::chrono::steady_clock::now() > deadline) {
                kill(pid_, SIGKILL);
                waitpid(pid_, &status, 0);
                break;
            }
            std::this_thread::sleep_for(10ms);
        }
        exited_ = true;
        if (reader_.joinable()) reader_.join();
        if (WIFEXITED(status)) result.exitCode = WEXITSTATUS(status);
        std::lock_guard<std::mutex> lock(mutex_);
        result.out = out_;
        result.err = err_;
        return result;
    }

private:
    pid_t pid_ = -1;
    bool exited_ = false;
    std::thread reader_;
    std::mutex mutex_;
    std::string out_;
    std::string err_;
    size_t cursor_ = 0;

    // Until both ends close (a pseudo-terminal reports EIO instead of end of file).
    void collect(std::array<int, 2> fds) {
        pollfd p[2] = {{fds[0], POLLIN, 0}, {fds[1], POLLIN, 0}};
        int open = 2;
        while (open > 0) {
            if (poll(p, 2, 100) <= 0) continue;
            for (int i = 0; i < 2; ++i) {
                if (p[i].fd < 0 || !(p[i].revents & (POLLIN | POLLHUP | POLLERR))) continue;
                char buf[4096];
                const ssize_t n = read(p[i].fd, buf, sizeof(buf));
                if (n > 0) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    (i == 0 ? out_ : err_).append(buf, static_cast<size_t>(n));
                } else {
                    close(p[i].fd);
                    p[i].fd = -1;
                    --open;
                }
            }
        }
    }
};

Result run(const std::string& tool, const std::vector<std::string>& args,
           const Options& options = {}) {
    return Tool(tool, args, options).wait();
}

// One run against `arm`, cleared for a new opener first, with `input` on stdin.
Result runOn(mc::fake::PtyArm& arm, const std::string& tool, const std::vector<std::string>& args,
             const std::string& input = "") {
    arm.unlock();
    return run(tool, args, {{{"MYCOBOT_PORT", arm.path()}}, input});
}

bool has(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

int servo(const mc::fake::PtyArm& arm, int joint, uint8_t address) {
    return arm.state().servoData.at({joint, address});
}

}  // namespace

// ---- Device overrides --------------------------------------------------------------

// Every program reports the device it was given, not its default: the variable was used.
TEST(Tools, SerialPortFromMycobotPort) {
    for (const auto& [tool, args] : std::map<std::string, std::vector<std::string>>{
             {"park", {}}, {"zero", {}}, {"pid_tool", {"read"}}, {"temps_lcd", {}}}) {
        SCOPED_TRACE(tool);
        const Result r =
            run(tool, args, {{{"MYCOBOT_PORT", MISSING}, {"MYCOBOT_LCD_BUS", MISSING}}});
        EXPECT_EQ(r.exitCode, 1);
        EXPECT_TRUE(has(r.err, std::string("could not open ") + MISSING)) << r.err;
    }
}

TEST(Tools, JoystickFromMycobotJoystick) {
    for (const std::string tool : {"joystick_all_joints", "joystick_display"}) {
        SCOPED_TRACE(tool);
        const Result r = run(tool, {"--debug"}, {{{"MYCOBOT_JOYSTICK", MISSING}}});
        EXPECT_EQ(r.exitCode, 1);
        EXPECT_TRUE(has(r.err, std::string("could not open ") + MISSING)) << r.err;
    }
}

TEST(Tools, LcdBusFromMycobotLcdBus) {
    const Result r =
        run("temps_lcd", {}, {{{"MYCOBOT_PORT", MISSING}, {"MYCOBOT_LCD_BUS", MISSING}}});
    EXPECT_TRUE(has(r.out, std::string("LCD not available (") + MISSING)) << r.out;
}

// ---- park, zero --------------------------------------------------------------------

TEST(Tools, ParkSendsTheParkPoseOnce) {
    mc::fake::PtyArm arm;
    arm.serve();
    const Result r = runOn(arm, "park", {});
    EXPECT_EQ(r.exitCode, 0) << r.err;
    EXPECT_EQ(arm.received(mc::protocol::SEND_ANGLES).size(), 1u);
    EXPECT_EQ(arm.state().angles, mc::PARK_ANGLES);
    EXPECT_TRUE(has(r.out, "Before: [0.00, 0.00, 0.00, 0.00, 0.00, 0.00]")) << r.out;
    EXPECT_TRUE(has(r.out, "After: [0.00, 131.00, -148.00")) << r.out;
}

TEST(Tools, ZeroSendsTheZeroPoseOnce) {
    mc::fake::PtyArm arm;
    arm.update([](auto& s) { s.angles = mc::PARK_ANGLES; });
    arm.serve();
    const Result r = runOn(arm, "zero", {});
    EXPECT_EQ(r.exitCode, 0) << r.err;
    EXPECT_EQ(arm.received(mc::protocol::SEND_ANGLES).size(), 1u);
    EXPECT_EQ(arm.state().angles, mc::ZERO_ANGLES);
    EXPECT_TRUE(has(r.out, "Before: [0.00, 131.00")) << r.out;
    EXPECT_TRUE(has(r.out, "After: [0.00, 0.00, 0.00, 0.00, 0.00, 0.00]")) << r.out;
}

// ---- pid_tool ----------------------------------------------------------------------

class PidTool : public ::testing::Test {
protected:
    void SetUp() override { arm_.serve(); }

    Result pidTool(const std::vector<std::string>& args, const std::string& input = "") {
        return runOn(arm_, "pid_tool", args, input);
    }

    mc::fake::PtyArm arm_;
};

TEST_F(PidTool, ReadShowsCurrentAndFactory) {
    Result r = pidTool({"read"});
    EXPECT_EQ(r.exitCode, 0) << r.err;
    EXPECT_TRUE(has(r.out, "J1      P  15  I   0  D  20      P  15  I   0  D  20\n")) << r.out;

    arm_.update([](auto& s) { s.servoData[{3, mc::servo::ADDR_P}] = 40; });
    r = pidTool({"read"});
    EXPECT_EQ(r.exitCode, 0);
    EXPECT_TRUE(has(r.out, "J3      P  40  I   0  D   8      P  32  I   0  D   8   <- differs"));
    EXPECT_TRUE(has(r.out, "1 joint(s) differ from the factory table."));

    arm_.update([](auto& s) { s.brokenServos = {5}; });
    r = pidTool({"read"});
    EXPECT_EQ(r.exitCode, 1);
    EXPECT_TRUE(has(r.out, "J5      (no answer)")) << r.out;
    EXPECT_TRUE(has(r.out, "Some joints did not answer."));
}

TEST_F(PidTool, SetWritesOnlyAfterYes) {
    Result r = pidTool({"set", "2", "40", "0", "8"}, "no\n");
    EXPECT_EQ(r.exitCode, 1);
    EXPECT_TRUE(has(r.out, "J2 now: P  32  I   0  D   8\nJ2 new: P  40  I   0  D   8")) << r.out;
    EXPECT_TRUE(has(r.out, "Aborted - nothing was changed."));
    EXPECT_EQ(servo(arm_, 2, mc::servo::ADDR_P), 32);

    r = pidTool({"set", "2", "40", "0", "8"}, "YES\n");
    EXPECT_EQ(r.exitCode, 0) << r.out;
    EXPECT_TRUE(has(r.out, "Done: J2 verified."));
    EXPECT_EQ(servo(arm_, 2, mc::servo::ADDR_P), 40);

    r = pidTool({"set", "2", "40", "0", "8"});
    EXPECT_EQ(r.exitCode, 0);
    EXPECT_TRUE(has(r.out, "J2 already has these values - nothing to do."));
}

TEST_F(PidTool, SetFailures) {
    Result r = pidTool({"set", "2", "99", "0", "8"});  // above 1.5 x factory: refused at once
    EXPECT_EQ(r.exitCode, 1);
    EXPECT_TRUE(has(r.err, "Error: ")) << r.err;

    arm_.update([](auto& s) { s.ignoreWrites = true; });
    r = pidTool({"set", "2", "40", "0", "8"}, "YES\n");
    EXPECT_EQ(r.exitCode, 1);
    EXPECT_TRUE(has(r.out, "!!! Some values could not be verified. Run `read`.")) << r.out;

    arm_.update([](auto& s) { s.brokenServos = {2}; });
    r = pidTool({"set", "2", "40", "0", "8"});
    EXPECT_EQ(r.exitCode, 1);
    EXPECT_TRUE(has(r.out, "J2 did not answer: nothing was changed."));
}

TEST_F(PidTool, FactoryRestoresOneJoint) {
    Result r = pidTool({"factory", "7"});
    EXPECT_EQ(r.exitCode, 1);
    EXPECT_TRUE(has(r.out, "joint must be 1..6"));

    arm_.update([](auto& s) { s.servoData[{3, mc::servo::ADDR_D}] = 10; });
    r = pidTool({"factory", "3"}, "YES\n");
    EXPECT_EQ(r.exitCode, 0) << r.out;
    EXPECT_EQ(servo(arm_, 3, mc::servo::ADDR_D), 8);

    r = pidTool({"bogus"});
    EXPECT_EQ(r.exitCode, 1);
    EXPECT_TRUE(has(r.out, "Usage:"));
}

TEST_F(PidTool, AllAsksEveryJointThenOneYes) {
    // J1: two invalid answers, then Enter (keep); J2..J5 kept; J6 changed.
    Result r = pidTool({"all"}, "abc\n99 0 0\n\n\n\n\n\n30 0 30\nYES\n");
    EXPECT_EQ(r.exitCode, 0) << r.out;
    EXPECT_TRUE(has(r.out, "Please type exactly three whole numbers"));
    EXPECT_TRUE(has(r.out, " - try again"));
    EXPECT_TRUE(has(r.out, "J6      P  25  I   0  D  25      P  30  I   0  D  30   <- change"));
    EXPECT_TRUE(has(r.out, "  J6 verified\nDone."));
    EXPECT_EQ(servo(arm_, 6, mc::servo::ADDR_P), 30);

    r = pidTool({"all"}, "\n\n\n\n\n\n");
    EXPECT_EQ(r.exitCode, 0);
    EXPECT_TRUE(has(r.out, "Nothing to change."));

    r = pidTool({"all"}, "\nq\n");
    EXPECT_EQ(r.exitCode, 1);
    EXPECT_TRUE(has(r.out, "Aborted - nothing was changed."));

    arm_.update([](auto& s) { s.ignoreWrites = true; });
    r = pidTool({"all"}, "\n\n\n\n\n25 0 25\nYES\n");
    EXPECT_EQ(r.exitCode, 1);
    EXPECT_TRUE(has(r.out, "!!! J6 could not be verified")) << r.out;
    EXPECT_TRUE(has(r.out, "Some joints failed. Run `read`."));

    arm_.update([](auto& s) { s.brokenServos = {4}; });
    r = pidTool({"all"});
    EXPECT_EQ(r.exitCode, 1);
    EXPECT_TRUE(has(r.out, "J4 did not answer: nothing was changed."));
}

TEST_F(PidTool, Menu) {
    const std::string input =
        "1\n"                        // read
        "foo\n"                      // unknown
        "\n"                         // nothing
        "2\nx\n7\n2\n40 0 8\nYES\n"  // set: two bad joints, then J2
        "4\n2\nYES\n"                // factory J2
        "set\n\n"                    // set, cancelled at the joint
        "2\n2\n\n"                   // set J2, Enter: nothing changed
        "factory\nq\n"               // factory, cancelled
        "3\nq\n"                     // all, quit at the first joint
        "quit\n";
    Result r = pidTool({}, input);
    EXPECT_EQ(r.exitCode, 0) << r.out;
    EXPECT_TRUE(has(r.out, "Joint   current                  factory"));
    EXPECT_TRUE(has(r.out, "Unknown command \"foo\" - type 1-4 or q"));
    EXPECT_TRUE(has(r.out, "Please type a joint number from 1 to 6"));
    EXPECT_TRUE(has(r.out, "Done: J2 verified."));
    EXPECT_TRUE(has(r.out, "Nothing was changed."));
    EXPECT_TRUE(has(r.out, "Bye."));
    EXPECT_EQ(servo(arm_, 2, mc::servo::ADDR_P), 32);  // set to 40, then back to factory

    // A failing command does not end the menu; end of input does.
    arm_.update([](auto& s) {
        s.moving = true;
        s.brokenServos = {3};
    });
    r = pidTool({}, "2\n2\n40 0 8\nYES\n2\n3\n");
    EXPECT_EQ(r.exitCode, 0);
    EXPECT_TRUE(has(r.out, "Error: arm is moving, PID values were not changed")) << r.out;
    EXPECT_TRUE(has(r.out, "J3 did not answer: nothing was changed."));
    EXPECT_TRUE(has(r.out, "Bye."));
}

// ---- temps_lcd ---------------------------------------------------------------------

TEST(Tools, TempsLcdReadsUntilInterrupted) {
    mc::fake::PtyArm arm;
    arm.update([](auto& s) { s.temps = {40, 41, 42, 70, 44, 45}; });
    arm.serve();
    // On a terminal, as it is used: stdout is then line-buffered, so lines arrive as printed.
    Tool tool("temps_lcd", {},
              {{{"MYCOBOT_PORT", arm.path()}, {"MYCOBOT_LCD_BUS", MISSING}}, "", true});
    EXPECT_TRUE(tool.next("Reading servo temperatures every 2 s."));
    EXPECT_TRUE(arm.waitFor(mc::protocol::GET_SERVO_TEMPS, 5s));

    arm.update([](auto& s) { s.silent = true; });
    EXPECT_TRUE(tool.next("No answer from the arm", 10s));
    tool.interrupt();
    const Result r = tool.wait();
    EXPECT_EQ(r.exitCode, 0) << r.err;
    EXPECT_TRUE(has(r.out, "Stopped. The last values stay on the LCD."));
}

// ---- joystick_all_joints, joystick_display ------------------------------------------

namespace {

// The joystick: a FIFO the program reads as its device. Opened read-write here, so the program's
// non-blocking open finds a writer and never sees end of file.
class FifoJoystick {
public:
    FifoJoystick() {
        char dir[] = "/tmp/joystickXXXXXX";
        if (mkdtemp(dir) == nullptr) return;
        dir_ = dir;
        path_ = dir_ + "/js0";
        mkfifo(path_.c_str(), 0600);
        fd_ = open(path_.c_str(), O_RDWR | O_NONBLOCK);
    }

    ~FifoJoystick() {
        close(fd_);
        unlink(path_.c_str());
        rmdir(dir_.c_str());
    }

    FifoJoystick(const FifoJoystick&) = delete;
    FifoJoystick& operator=(const FifoJoystick&) = delete;

    const std::string& path() const { return path_; }

    // `ms`: the event's time; by default a second after the previous one (outside every guard).
    void axis(uint8_t number, int16_t value, std::optional<uint32_t> ms = {}) {
        send(JS_EVENT_AXIS, number, value, ms);
    }
    void button(uint8_t number, int16_t value, std::optional<uint32_t> ms = {}) {
        send(JS_EVENT_BUTTON, number, value, ms);
    }
    void click(uint8_t number) {
        button(number, 1);
        button(number, 0);
    }
    uint32_t time() const { return time_; }

private:
    std::string dir_;
    std::string path_;
    int fd_ = -1;
    uint32_t time_ = 0;

    void send(uint8_t type, uint8_t number, int16_t value, std::optional<uint32_t> ms) {
        time_ = ms ? *ms : time_ + 1000;
        const js_event e{time_, value, type, number};
        [[maybe_unused]] const ssize_t n = write(fd_, &e, sizeof(e));
    }
};

constexpr uint8_t AXIS_J1 = 0;  // left stick X (no effect on the height: never refused)
constexpr uint8_t AXIS_J2 = 1;  // left stick Y
constexpr uint8_t TRIGGER = 4;  // analog trigger: never moves a joint
constexpr uint8_t BUTTON_A = 0;
constexpr uint8_t BUTTON_B = 1;
constexpr uint8_t BUTTON_X = 3;
constexpr uint8_t BUTTON_Y = 4;
constexpr uint8_t BUTTON_SPEED = 9;
constexpr int16_t FULL = 32767;

bool near(const mc::Angles& a, const mc::Angles& b, double tolerance) {
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > tolerance) return false;
    }
    return true;
}

// The same scenarios for both programs: their jogging logic and messages are the same.
class Joystick : public ::testing::TestWithParam<std::string> {
protected:
    void SetUp() override { arm_.serve(); }

    Options devices(Options options = {}) {
        arm_.unlock();
        options.env["MYCOBOT_PORT"] = arm_.path();
        options.env["MYCOBOT_JOYSTICK"] = js_.path();
        return options;
    }

    // Start the program in --debug and wait for its control loop.
    std::unique_ptr<Tool> start() {
        auto tool =
            std::make_unique<Tool>(GetParam(), std::vector<std::string>{"--debug"}, devices());
        EXPECT_TRUE(tool->next("Ready.", 10s));
        return tool;
    }

    // Ctrl+C: the program stops the arm and exits 0.
    void quit(Tool& tool) {
        arm_.update([](auto& s) { s.silent = false; });
        tool.interrupt();
        const Result r = tool.wait();
        EXPECT_EQ(r.exitCode, 0) << r.err;
        EXPECT_TRUE(has(r.out, "Arm stopped. Final angles: ["));
    }

    mc::fake::PtyArm arm_;
    FifoJoystick js_;
};

}  // namespace

TEST_P(Joystick, JogsUpToTheLimitAndStops) {
    auto tool = start();
    js_.axis(TRIGGER, FULL);  // ignored
    js_.axis(AXIS_J1, FULL);  // J1 is inverted: toward its minimum
    EXPECT_TRUE(tool->next(">>> jogging joints [1]"));
    EXPECT_TRUE(tool->next(">>> J1 at limit", 20s));
    EXPECT_NEAR(arm_.state().angles[0], mc::JOINT_LIMITS[0].min, 0.5);
    js_.axis(AXIS_J1, 0);
    EXPECT_TRUE(tool->next(">>> stop"));
    EXPECT_TRUE(arm_.waitFor(mc::protocol::STOP, 1s));
    quit(*tool);
}

TEST_P(Joystick, HoldsIdleAndReleasedJoints) {
    auto tool = start();
    js_.axis(AXIS_J1, FULL);
    EXPECT_TRUE(tool->next(">>> jogging joints [1]"));
    arm_.update([](auto& s) { s.angles[2] += 5; });  // J3 sags while J1 moves
    EXPECT_TRUE(tool->next("hold J3: drift +5.0"));
    js_.axis(AXIS_J2, FULL);
    EXPECT_TRUE(tool->next(">>> jogging joints [1, 2]"));
    js_.axis(AXIS_J2, 0);  // J2 released while J1 still moves: pinned where it is
    EXPECT_TRUE(tool->next(">>> jogging joints [1]"));
    js_.axis(AXIS_J1, 0);
    EXPECT_TRUE(tool->next(">>> stop"));
    quit(*tool);
}

TEST_P(Joystick, ButtonsGripperAndSpeedMode) {
    auto tool = start();
    js_.click(BUTTON_A);
    EXPECT_TRUE(tool->next(">>> close gripper"));
    js_.click(BUTTON_B);
    EXPECT_TRUE(tool->next(">>> open gripper"));
    EXPECT_TRUE(arm_.waitFor(mc::protocol::SET_GRIPPER_STATE, 1s));

    // A stick touched together with button 9 is blocked until it is back at the centre.
    js_.button(BUTTON_SPEED, 1);
    js_.axis(AXIS_J1, FULL, js_.time() + 10);
    EXPECT_TRUE(tool->next(">>> speed mode: 50%"));
    EXPECT_TRUE(tool->next("axis   0 blocked (fired with button 9)"));
    js_.button(BUTTON_SPEED, 0);
    js_.axis(AXIS_J1, 0);
    EXPECT_TRUE(tool->next("axis   0 unblocked"));
    EXPECT_GE(arm_.received(mc::protocol::SET_COLOR).size(), 2u);  // start, then 50%
    quit(*tool);
}

TEST_P(Joystick, HomingCanBeCancelledOrAborted) {
    arm_.update([](auto& s) { s.angles = {40, -30, 20, 0, 10, 0}; });
    auto tool = start();
    js_.click(BUTTON_X);
    EXPECT_TRUE(tool->next(">>> homing (click X again or touch a stick to abort)"));
    EXPECT_TRUE(tool->next(">>> home reached", 20s));
    EXPECT_TRUE(near(arm_.state().angles, mc::ZERO_ANGLES, 1.0));

    arm_.update([](auto& s) { s.angles = {150, 0, 0, 0, 0, 0}; });
    js_.click(BUTTON_X);
    EXPECT_TRUE(tool->next(">>> homing ("));
    js_.click(BUTTON_X);
    EXPECT_TRUE(tool->next(">>> homing cancelled"));

    js_.click(BUTTON_X);
    EXPECT_TRUE(tool->next(">>> homing ("));
    js_.axis(AXIS_J1, FULL);
    EXPECT_TRUE(tool->next(">>> homing aborted by stick/bumper input"));
    js_.axis(AXIS_J1, 0);
    quit(*tool);
}

TEST_P(Joystick, NoFeedbackStopsTheArm) {
    auto tool = start();
    arm_.update([](auto& s) { s.silent = true; });
    js_.axis(AXIS_J1, FULL);
    EXPECT_TRUE(tool->next("!!! no valid angle feedback, stopping"));
    js_.axis(AXIS_J1, 0);

    js_.click(BUTTON_X);
    EXPECT_TRUE(tool->next("!!! no valid angle feedback, homing aborted", 20s));
    quit(*tool);
}

INSTANTIATE_TEST_SUITE_P(Tools, Joystick,
                         ::testing::Values("joystick_all_joints", "joystick_display"));

// ---- joystick_display only: park, temperatures, the status screen ----------------------

class JoystickDisplay : public Joystick {};

TEST_P(JoystickDisplay, ParkReleasesThenLocks) {
    auto tool = start();
    js_.click(BUTTON_Y);
    EXPECT_TRUE(tool->next(">>> parking (click Y again or touch a stick to abort)"));
    EXPECT_TRUE(tool->next(">>> park position reached", 20s));
    EXPECT_TRUE(tool->next(">>> parked - joints released (click Y to lock them)"));
    EXPECT_TRUE(arm_.waitFor(mc::protocol::RELEASE_ALL_SERVOS, 1s));

    js_.click(BUTTON_A);  // ignored while parked
    EXPECT_TRUE(tool->next(">>> parked: joints are free - click Y to lock them first"));
    js_.axis(AXIS_J1, FULL);
    EXPECT_TRUE(tool->next(">>> parked: joints are free - click Y to lock them first"));
    js_.axis(AXIS_J1, 0);
    js_.click(BUTTON_Y);
    EXPECT_TRUE(tool->next(">>> joints locked."));
    EXPECT_TRUE(arm_.waitFor(mc::protocol::POWER_ON, 1s));
    quit(*tool);
}

TEST_P(JoystickDisplay, ParkCanBeCancelledOrAborted) {
    const auto far = [](auto& s) { s.angles = {-150, 0, 0, 0, 0, 0}; };  // several steps away
    arm_.update(far);
    auto tool = start();
    js_.click(BUTTON_X);
    EXPECT_TRUE(tool->next(">>> homing ("));
    js_.click(BUTTON_Y);  // parking replaces homing
    EXPECT_TRUE(tool->next(">>> homing cancelled"));
    EXPECT_TRUE(tool->next(">>> parking ("));
    js_.click(BUTTON_Y);
    EXPECT_TRUE(tool->next(">>> parking cancelled"));

    arm_.update(far);
    js_.click(BUTTON_Y);
    EXPECT_TRUE(tool->next(">>> parking ("));
    js_.click(BUTTON_X);
    EXPECT_TRUE(tool->next(">>> parking cancelled"));

    arm_.update(far);
    js_.click(BUTTON_Y);
    EXPECT_TRUE(tool->next(">>> parking ("));
    js_.axis(AXIS_J1, FULL);
    EXPECT_TRUE(tool->next(">>> parking aborted by stick/bumper input"));
    js_.axis(AXIS_J1, 0);

    arm_.update([](auto& s) { s.silent = true; });
    js_.click(BUTTON_Y);
    EXPECT_TRUE(tool->next("!!! no valid angle feedback, parking aborted", 20s));
    quit(*tool);
}

TEST_P(JoystickDisplay, TemperatureWarnings) {
    auto tool = start();
    arm_.update([](auto& s) { s.temps = {40, 41, 42, 70, 44, 45}; });
    EXPECT_TRUE(tool->next("!!! servo J:4 HOT (70 C) - let the arm rest", 10s));
    arm_.update([](auto& s) { s.temps = {40, 58, 42, 44, 44, 45}; });
    EXPECT_TRUE(tool->next(">>> servo J:2 warm (58 C)", 10s));
    arm_.update([](auto& s) { s.temps = {40, 41, 42, 44, 44, 45}; });
    EXPECT_TRUE(tool->next(">>> servo temperatures normal (max 45 C)", 10s));
    quit(*tool);
}

TEST_P(JoystickDisplay, StatusScreenOnATerminal) {
    Options options;
    options.terminal = true;
    Tool tool(GetParam(), {}, devices(options));
    EXPECT_TRUE(tool.next("myCobot 280 - joystick control", 10s));
    js_.axis(AXIS_J1, FULL);
    EXPECT_TRUE(tool.next("JOGGING", 10s));
    js_.axis(AXIS_J1, 0);
    arm_.update([](auto& s) { s.silent = true; });  // idle reads fail: shown on the screen
    EXPECT_TRUE(tool.next("NO ANGLE FEEDBACK", 10s));
    quit(tool);
}

INSTANTIATE_TEST_SUITE_P(Tools, JoystickDisplay, ::testing::Values("joystick_display"));
