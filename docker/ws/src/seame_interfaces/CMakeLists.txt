cmake_minimum_required(VERSION 3.10)
project(seame_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointTarget.msg"
  "msg/ArmCommand.msg"
  "msg/ServoTemperatures.msg"
  "srv/AddTwoInts.srv"
  "action/Fibonacci.action"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
