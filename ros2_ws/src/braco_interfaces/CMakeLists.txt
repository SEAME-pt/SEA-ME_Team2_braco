cmake_minimum_required(VERSION 3.10)
project(braco_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointTarget.msg"
  "msg/ArmCommand.msg"
  "msg/ServoTemperatures.msg"
  "msg/ServoVoltages.msg"
  "srv/TakeControl.srv"
  "action/MoveToPose.action"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
