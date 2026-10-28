// Copyright 2026 SEA:ME Team 2
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// ControlLease - who may move the arm (docs/decisions/ADR-0002-unique-control.md).
//
// One owner at a time ("web", "joystick", "script", "mission"; "" = free). arm_driver
// obeys motion only from the owner; STOP is not its business, it always passes.
//   take     free, already ours, or force -> ours; otherwise refused with the owner's name
//   release  only by the owner
//   touch    the owner sent motion: the lease lasts
//   expire   no motion for `idle`: free again
// No ROS and no clock of its own (the caller passes `now`), so it is unit tested.

#ifndef BRACO_DRIVER__CONTROL_HPP_
#define BRACO_DRIVER__CONTROL_HPP_

#include <chrono>
#include <string>

namespace braco_driver
{

class ControlLease
{
public:
  using Clock = std::chrono::steady_clock;

  struct Result
  {
    bool success;
    std::string owner;      // owner after the call ("" = free)
    std::string previous;   // owner before the call
  };

  Result take(const std::string & source, bool force, Clock::time_point now)
  {
    const std::string previous = owner_;
    if (source.empty()) {return {false, owner_, previous};}
    if (!owner_.empty() && owner_ != source && !force) {return {false, owner_, previous};}
    owner_ = source;
    last_ = now;
    return {true, owner_, previous};
  }

  bool release(const std::string & source)
  {
    if (source.empty() || owner_ != source) {return false;}
    owner_.clear();
    return true;
  }

  // True when motion from `source` may move the arm.
  bool accepts(const std::string & source) const
  {
    return !owner_.empty() && owner_ == source;
  }

  void touch(Clock::time_point now) {last_ = now;}

  // Frees the control after `idle` without touch(); returns the owner it dropped ("" = none).
  std::string expireIdle(Clock::time_point now, Clock::duration idle)
  {
    if (owner_.empty() || now - last_ <= idle) {return "";}
    std::string dropped;
    dropped.swap(owner_);
    return dropped;
  }

  const std::string & owner() const {return owner_;}

private:
  std::string owner_;
  Clock::time_point last_{};
};

}  // namespace braco_driver

#endif  // BRACO_DRIVER__CONTROL_HPP_
