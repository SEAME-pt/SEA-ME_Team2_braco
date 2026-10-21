// joystick.cpp - implementation of input::Joystick (see joystick.hpp).

#include "joystick.hpp"

#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace input
{

Joystick::Joystick(const std::string & device)
{
  fd_ = open(device.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("could not open " + device + ": " + std::strerror(errno));
  }
}

Joystick::~Joystick()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}

std::vector<Event> Joystick::readEvents()
{
  std::vector<Event> events;
  while (true) {
    js_event raw{};
    ssize_t n = read(fd_, &raw, sizeof(raw));
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) break;  // no more events
      throw std::runtime_error(std::string("joystick read failed: ") + std::strerror(errno));
    }
    if (n != static_cast<ssize_t>(sizeof(raw))) break;  // partial/no data
    if (raw.type & JS_EVENT_INIT) continue;             // initial state, skip
    events.push_back({raw.time, raw.value, raw.type, raw.number});
  }
  return events;
}

}  // namespace input
