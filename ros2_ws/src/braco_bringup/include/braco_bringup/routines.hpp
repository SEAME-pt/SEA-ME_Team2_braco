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

// The routines of the zero and park programs, in the arm_moves library so the tests can run them
// against a fake arm_driver. Both return true on success; the programs turn that into an exit code.

#ifndef BRACO_BRINGUP__ROUTINES_HPP_
#define BRACO_BRINGUP__ROUTINES_HPP_

#include "braco_bringup/arm_moves.hpp"

namespace braco_bringup
{

// Boot routine: leave rest mode, lock the joints and send the arm to mycobot::ZERO_ANGLES.
bool zero(ArmMoves & arm);

// Shutdown routine: fold the arm into mycobot::PARK_ANGLES, release all joints, enter rest mode.
bool park(ArmMoves & arm);

}  // namespace braco_bringup

#endif  // BRACO_BRINGUP__ROUTINES_HPP_
