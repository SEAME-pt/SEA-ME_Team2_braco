# braco_mission

Mission layer of the braco arm: tasks such as pick-and-place and drawing with a pen, written as a state machine
that asks MoveIt 2 for motions. **Empty for now**: the package reserves the name; its content arrives with issue
**#43** (reach a Cartesian goal pose through MoveIt 2).

The mission never talks to `arm_driver` or the serial port directly: it only uses MoveIt 2
(`braco_moveit_config`), so the same code runs on the real arm and in simulation (#37). Python or C++ is decided
with #43; change `package.xml` and `CMakeLists.txt` then.
