// joystick.hpp - minimal reader for Linux joystick devices (/dev/input/jsX).
//
// Reads the kernel joystick API (linux/joystick.h) in non-blocking mode.
// The implementation lives in joystick.cpp.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace input
{

constexpr uint8_t TYPE_BUTTON = 0x01;  // JS_EVENT_BUTTON
constexpr uint8_t TYPE_AXIS = 0x02;    // JS_EVENT_AXIS

struct Event
{
  uint32_t timeMs;  // kernel timestamp in milliseconds
  int16_t value;    // axis: -32767..32767, button: 0 = released, 1 = pressed
  uint8_t type;     // TYPE_BUTTON or TYPE_AXIS
  uint8_t number;   // axis or button number
};

class Joystick
{
public:
  static constexpr const char * DEFAULT_DEVICE = "/dev/input/js0";

  // Opens the device. Throws std::runtime_error on failure.
  explicit Joystick(const std::string & device = DEFAULT_DEVICE);
  ~Joystick();

  Joystick(const Joystick &) = delete;
  Joystick & operator=(const Joystick &) = delete;

  // Returns all pending events without blocking. Synthetic "initial state"
  // events sent by the kernel on open are skipped.
  // Throws std::runtime_error if the device disappears (e.g. unplugged).
  std::vector<Event> readEvents();

private:
  int fd_ = -1;
};

}  // namespace input
