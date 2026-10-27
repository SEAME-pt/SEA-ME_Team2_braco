# braco_moveit_config

MoveIt 2 configuration of the braco arm. **Empty for now**: the package reserves the name; its content (the MoveIt 2
configuration for the arm) is planned.

Planned content in `config/`: `braco.srdf` (planning group, end effector, disabled collisions),
`kinematics.yaml`, `joint_limits.yaml` (the same limits as `JOINT_LIMITS` in `mycobot.hpp`) and the controllers MoveIt sends
trajectories to. Built from the model in `braco_description` (#25).

On the Jetson Nano, `move_group` needs 300 to 600 MB and a full core while planning: run it on the PC; it reaches
the arm on the Jetson through DDS.
