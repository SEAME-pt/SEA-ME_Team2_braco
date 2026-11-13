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

// Gives the arm control back when it goes out of scope (ADR-0004): every return of a mission
// step releases it, also on failure. Shared by move_to_pose and move_cartesian.

#ifndef BRACO_MISSION__CONTROL_GUARD_HPP_
#define BRACO_MISSION__CONTROL_GUARD_HPP_

#include "braco_mission/mission.hpp"

namespace braco_mission
{

class ControlGuard
{
public:
  explicit ControlGuard(ArmBackend & arm)
  : arm_(arm) {}
  ~ControlGuard() {if (held_) {arm_.release_control();}}
  ControlGuard(const ControlGuard &) = delete;
  ControlGuard & operator=(const ControlGuard &) = delete;
  void hold() {held_ = true;}

private:
  ArmBackend & arm_;
  bool held_ = false;
};

}  // namespace braco_mission

#endif  // BRACO_MISSION__CONTROL_GUARD_HPP_
