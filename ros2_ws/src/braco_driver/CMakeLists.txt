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

# The two nodes in a library, so the tests can run them in-process; the programs only hold main()
add_library(braco_driver_nodes STATIC src/arm_driver_node.cpp src/temperature_monitor_node.cpp)
target_include_directories(braco_driver_nodes PUBLIC include)
target_link_libraries(braco_driver_nodes PUBLIC
  mycobot_cpp::mycobot_cpp rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${std_msgs_TARGETS} ${std_srvs_TARGETS} ${diagnostic_msgs_TARGETS}
  ${braco_interfaces_TARGETS})

foreach(program arm_driver temperature_monitor)
  add_executable(${program} src/${program}.cpp)
  target_link_libraries(${program} braco_driver_nodes)
endforeach()

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

  # ArmDriver on a fake arm (PtyArm, a pseudo-terminal) with a client node in the same process.
  # Isolated: its own ROS graph, so a real arm_driver on the same machine is never seen.
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(test_arm_driver test/test_arm_driver.cpp TIMEOUT 120)
  target_link_libraries(test_arm_driver braco_driver_nodes util)
endif()

ament_package()
