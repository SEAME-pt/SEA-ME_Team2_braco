cmake_minimum_required(VERSION 3.10)
project(seame_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mycobot_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(seame_interfaces REQUIRED)

add_executable(arm_driver src/arm_driver.cpp)
target_link_libraries(arm_driver
  mycobot_cpp::mycobot_cpp rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${seame_interfaces_TARGETS})

add_executable(temperature_monitor src/temperature_monitor.cpp)
target_link_libraries(temperature_monitor
  mycobot_cpp::mycobot_cpp rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS} ${seame_interfaces_TARGETS})

add_executable(joystick_teleop src/joystick_teleop.cpp)
target_include_directories(joystick_teleop PRIVATE include)
target_link_libraries(joystick_teleop
  mycobot_cpp::mycobot_cpp rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${seame_interfaces_TARGETS})

install(TARGETS arm_driver temperature_monitor joystick_teleop
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# Boot/shutdown routine (container arm-routine): zero.py at start, park.py before stopping
install(PROGRAMS scripts/zero.py scripts/park.py scripts/arm_routine.sh
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/arm_moves.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_teleop_logic test/test_teleop_logic.cpp)
  target_include_directories(test_teleop_logic PRIVATE include)
  target_link_libraries(test_teleop_logic mycobot_cpp::mycobot_cpp)
endif()

ament_package()
