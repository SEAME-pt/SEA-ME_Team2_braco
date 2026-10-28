cmake_minimum_required(VERSION 3.10)
project(braco_teleop)

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
find_package(braco_interfaces REQUIRED)

# The node in a library, so the tests can run it in-process; the program only holds main()
add_library(braco_teleop_node STATIC src/joystick_teleop_node.cpp)
target_include_directories(braco_teleop_node PUBLIC include)
target_link_libraries(braco_teleop_node PUBLIC
  mycobot_cpp::mycobot_cpp rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${braco_interfaces_TARGETS})

add_executable(joystick_teleop src/joystick_teleop.cpp)
target_link_libraries(joystick_teleop braco_teleop_node)

install(TARGETS joystick_teleop
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_teleop_logic test/test_teleop_logic.cpp)
  target_link_libraries(test_teleop_logic mycobot_cpp::mycobot_cpp)
endif()

ament_package()
