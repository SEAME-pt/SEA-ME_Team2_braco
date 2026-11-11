cmake_minimum_required(VERSION 3.10)
project(braco_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(braco_interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# The plugin: a shared library with no main, loaded by ros2_control_node at run time
add_library(braco_hardware SHARED src/braco_system.cpp)
target_include_directories(braco_hardware PRIVATE include)
target_link_libraries(braco_hardware PUBLIC
  hardware_interface::hardware_interface pluginlib::pluginlib
  rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle ${sensor_msgs_TARGETS}
  ${braco_interfaces_TARGETS} ${std_msgs_TARGETS})

# Registers braco_hardware.xml with pluginlib: how ros2_control_node finds the class by name
pluginlib_export_plugin_description_file(hardware_interface braco_hardware.xml)

install(TARGETS braco_hardware
  LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_cppcheck skips every file with cppcheck >= 2.0; make cppcheck runs cppcheck directly instead
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
