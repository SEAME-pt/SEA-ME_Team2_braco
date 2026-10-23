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

// AutoRest - when temperature_monitor puts the arm to rest
// (docs/ADR-0003-rest-per-temp.md).
//
// An episode starts with the first reading at TEMP_HOT_C or above and ends when every joint
// is below REARM_BELOW_C. The gap between the two is hysteresis: a servo hovering at 64/65 C
// (1 C steps, one reading every 2 s) stays in one episode instead of starting a new one, and a
// new park, on every upward crossing. The arm is put to rest once per episode: an operator who
// resumes a hot arm on purpose (make resume) is not overruled on every reading. Leaving rest is
// never automatic. No ROS, so it is unit tested.

#ifndef BRACO_DRIVER__AUTO_REST_HPP_
#define BRACO_DRIVER__AUTO_REST_HPP_

#include "mycobot.hpp"

namespace braco_driver
{

class AutoRest
{
public:
  // A hot episode ends only when the hottest joint is below this: really cooled down.
  static constexpr int REARM_BELOW_C = 60;
  static_assert(
    mycobot::TEMP_WARM_C < REARM_BELOW_C && REARM_BELOW_C < mycobot::TEMP_HOT_C,
    "REARM_BELOW_C must lie between TEMP_WARM_C and TEMP_HOT_C");

  // One valid reading, as the temperature of its hottest joint. True when the arm must be
  // put to rest now.
  bool update(int hottest_celsius)
  {
    if (hottest_celsius < REARM_BELOW_C) {
      episode_ = false;
      return false;
    }
    if (hottest_celsius < mycobot::TEMP_HOT_C || episode_) {return false;}
    episode_ = true;
    return true;
  }

  // The rest request could not be sent: try again on the next hot reading.
  void retry() {episode_ = false;}

  // True while the current episode already put the arm to rest.
  bool active() const {return episode_;}

private:
  bool episode_ = false;
};

}  // namespace braco_driver

#endif  // BRACO_DRIVER__AUTO_REST_HPP_
