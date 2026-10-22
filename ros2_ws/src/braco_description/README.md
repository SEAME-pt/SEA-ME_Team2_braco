# braco_description

Model of the myCobot 280 for the braco stack. **Empty for now**: the package exists so every team member uses the
same name and folder; its content arrives with issue **#25** (validate the URDF, joint limits and frames).

Planned content:

| Folder | Content |
|---|---|
| `urdf/` | `braco.urdf.xacro` (links, joints, limits) and `braco.ros2_control.xacro` (the `<ros2_control>` tag) |
| `meshes/` | Visual and collision meshes |
| `rviz/` | `braco.rviz`, the view used by `braco_bringup` |

Rules for the model: joint limits match `JOINT_LIMITS` in `ros2_ws/src/mycobot_cpp/includes/mycobot.hpp`; joint names are the
ones `/joint_states` uses on the real arm and in simulation (one interface for both, #37). When the files are
added, install them from `CMakeLists.txt`:

```cmake
install(DIRECTORY urdf meshes rviz DESTINATION share/${PROJECT_NAME})
```
