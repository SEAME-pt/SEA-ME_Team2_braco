// test_tools.cpp - GoogleTests for the stand-alone programs (srcs/park.cpp, zero.cpp, ...), run as
// the built binaries (TOOLS_DIR, from CMake) with their devices pointed at fakes through MYCOBOT_PORT,
// MYCOBOT_JOYSTICK and MYCOBOT_LCD_BUS. The arm is a PtyArm (pty_arm.hpp) answering like the real
// one. A program that returns from main() writes its coverage data like any other process.

#include <gtest/gtest.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include "mycobot.hpp"
#include "pty_arm.hpp"

namespace mc = mycobot;

namespace {

constexpr const char* MISSING = "/dev/does-not-exist";

struct Result {
    int exitCode = -1;  // -1: killed after the timeout
    std::string out;
    std::string err;
};

// Run TOOLS_DIR/<tool> with `args` and these variables added to the environment; kill it after
// `timeout`.
Result run(const std::string& tool, const std::vector<std::string>& args,
           const std::map<std::string, std::string>& env,
           std::chrono::milliseconds timeout = std::chrono::milliseconds(20000)) {
    int out[2];
    int err[2];
    if (pipe(out) != 0 || pipe(err) != 0) return {};
    const std::string path = std::string(TOOLS_DIR) + "/" + tool;

    const pid_t pid = fork();
    if (pid == 0) {
        dup2(out[1], STDOUT_FILENO);
        dup2(err[1], STDERR_FILENO);
        close(out[0]);
        close(err[0]);
        for (const auto& [name, value] : env) setenv(name.c_str(), value.c_str(), 1);
        std::vector<char*> argv = {const_cast<char*>(path.c_str())};
        for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
        argv.push_back(nullptr);
        execv(path.c_str(), argv.data());
        _exit(127);
    }
    close(out[1]);
    close(err[1]);

    Result result;
    pollfd fds[2] = {{out[0], POLLIN, 0}, {err[0], POLLIN, 0}};
    std::string* text[2] = {&result.out, &result.err};
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    int open = 2;
    while (open > 0) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (left.count() <= 0) {
            kill(pid, SIGKILL);
            break;
        }
        if (poll(fds, 2, static_cast<int>(left.count())) <= 0) continue;
        for (int i = 0; i < 2; ++i) {
            if (fds[i].fd < 0 || !(fds[i].revents & (POLLIN | POLLHUP))) continue;
            char buf[512];
            const ssize_t n = read(fds[i].fd, buf, sizeof(buf));
            if (n > 0) {
                text[i]->append(buf, static_cast<size_t>(n));
            } else {
                close(fds[i].fd);
                fds[i].fd = -1;
                --open;
            }
        }
    }
    for (const auto& f : fds) {
        if (f.fd >= 0) close(f.fd);
    }
    int status = 0;
    waitpid(pid, &status, 0);
    if (WIFEXITED(status)) result.exitCode = WEXITSTATUS(status);
    return result;
}

}  // namespace

// ---- Device overrides --------------------------------------------------------------

// Every program reports the device it was given, not its default: the variable was used.
TEST(Tools, SerialPortFromMycobotPort) {
    for (const auto& [tool, args] : std::map<std::string, std::vector<std::string>>{
             {"park", {}}, {"zero", {}}, {"pid_tool", {"read"}}, {"temps_lcd", {}}}) {
        SCOPED_TRACE(tool);
        const Result r = run(tool, args, {{"MYCOBOT_PORT", MISSING}, {"MYCOBOT_LCD_BUS", MISSING}});
        EXPECT_EQ(r.exitCode, 1);
        EXPECT_NE(r.err.find(std::string("could not open ") + MISSING), std::string::npos) << r.err;
    }
}

TEST(Tools, JoystickFromMycobotJoystick) {
    for (const std::string tool : {"joystick_all_joints", "joystick_display"}) {
        SCOPED_TRACE(tool);
        const Result r = run(tool, {"--debug"}, {{"MYCOBOT_JOYSTICK", MISSING}});
        EXPECT_EQ(r.exitCode, 1);
        EXPECT_NE(r.err.find(std::string("could not open ") + MISSING), std::string::npos) << r.err;
    }
}

TEST(Tools, LcdBusFromMycobotLcdBus) {
    const Result r =
        run("temps_lcd", {}, {{"MYCOBOT_PORT", MISSING}, {"MYCOBOT_LCD_BUS", MISSING}});
    EXPECT_NE(r.out.find(std::string("LCD not available (") + MISSING), std::string::npos) << r.out;
}

// ---- park ------------------------------------------------------------------------

TEST(Tools, ParkSendsTheParkPoseOnce) {
    mc::fake::PtyArm arm;
    arm.serve();
    const Result r = run("park", {}, {{"MYCOBOT_PORT", arm.path()}});
    EXPECT_EQ(r.exitCode, 0) << r.err;
    EXPECT_EQ(arm.received(mc::protocol::SEND_ANGLES).size(), 1u);
    EXPECT_EQ(arm.state().angles, mc::PARK_ANGLES);
    EXPECT_NE(r.out.find("Before: [0.00, 0.00, 0.00, 0.00, 0.00, 0.00]"), std::string::npos)
        << r.out;
    EXPECT_NE(r.out.find("After: [0.00, 131.00, -148.00"), std::string::npos) << r.out;
}
