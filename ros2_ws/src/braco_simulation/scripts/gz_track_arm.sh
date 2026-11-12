#!/bin/sh
# Copyright 2026 SEA:ME Team 2
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Turn on Track in the Gazebo window: the camera keeps looking at the arm, 0.25 m above its base.
# Waits for the window to subscribe to /gui/track (it takes seconds to open), then sends one
# message, so the user can stop tracking from the window afterwards. Run by launch/gazebo.launch.py
# and launch/gazebo_gui.launch.py. Without a window after 60 s it gives up: Track is only a comfort.

tries=0
until gz topic -i -t /gui/track 2>/dev/null | grep -q 'gz.msgs.CameraTrack'; do
    tries=$((tries + 1))
    if [ "$tries" -ge 60 ]; then
        echo "gz_track_arm: no Gazebo window on /gui/track, Track not set" >&2
        exit 0
    fi
    sleep 1
done

# The window subscribes before it has the scene: give it time to receive the arm.
sleep 3
gz topic -t /gui/track -m gz.msgs.CameraTrack \
    -p 'track_mode: TRACK track_target: {name: "braco"} track_offset: {z: 0.25} track_pgain: 1.0'
