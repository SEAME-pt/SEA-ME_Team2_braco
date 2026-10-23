cmake_minimum_required(VERSION 3.10)
project(braco_driver)

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
find_package(braco_interfaces REQUIRED)

add_executable(arm_driver src/arm_driver.cpp)
target_include_directories(arm_driver PRIVATE include)
target_link_libraries(arm_driver
  mycobot_cpp::mycobot_cpp rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${braco_interfaces_TARGETS})

add_executable(temperature_monitor src/temperature_monitor.cpp)
target_include_directories(temperature_monitor PRIVATE include)
target_link_libraries(temperature_monitor
  mycobot_cpp::mycobot_cpp rclcpp::rclcpp
  ${diagnostic_msgs_TARGETS} ${std_srvs_TARGETS} ${braco_interfaces_TARGETS})

install(TARGETS arm_driver temperature_monitor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # ControlLease and AutoRest: plain C++, no ROS graph needed
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control test/test_control.cpp)
  target_include_directories(test_control PRIVATE include)
  target_link_libraries(test_control mycobot_cpp::mycobot_cpp)
endif()

ament_package()
