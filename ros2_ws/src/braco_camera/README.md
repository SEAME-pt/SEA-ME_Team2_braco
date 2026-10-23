# braco_camera

`camera`: the eye-in-hand USB camera of the arm on ROS 2. It is the only process that opens the camera; the web
page (and later the vision nodes) subscribe to its topic.

| | Name |
|---|---|
| pub | `/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`, `format: jpeg`, sensor-data QoS) |

The camera is asked for MJPG, so its JPEG frames are published as they come, without decoding them (cheap on the
Nano); a camera that only gives raw frames is encoded here. Without the camera (unplugged, wrong device) the node
keeps running and tries again every `reopen_s`: the arm never depends on it.

| Parameter | Default | |
|---|---|---|
| `device` | `/dev/video0` | the launch sets it from `camera:=` (`CAMERA_DEVICE` in `docker/.env`) |
| `width` / `height` | `640` / `480` | asked of the camera, which may pick the closest it has |
| `fps` | `15.0` | the most it publishes |
| `jpeg_quality` | `80` | only for cameras without MJPG |
| `frame_id` | `camera_link` | |
| `reopen_s` | `2.0` | retry period without a camera |

Starts with the whole system (`make up`); by hand: `ros2 run braco_camera camera --ros-args -p device:=/dev/video0`.
In the `real` container the host's `/dev` is mounted at `/run/hostdev`, so the camera can be plugged in after
the container started ([docker/compose.yml](../../../docker/compose.yml)).
