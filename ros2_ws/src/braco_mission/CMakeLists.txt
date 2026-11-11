cmake_minimum_required(VERSION 3.10)
project(braco_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(braco_interfaces REQUIRED)

# The steps of a goal, no ROS (mission.cpp, cartesian.cpp): the node and the tests link it
add_library(braco_mission_core STATIC src/mission.cpp src/cartesian.cpp)
target_include_directories(braco_mission_core PUBLIC include)

add_executable(move_to_pose src/move_to_pose_node.cpp)
target_link_libraries(move_to_pose braco_mission_core
  rclcpp::rclcpp rclcpp_action::rclcpp_action ${geometry_msgs_TARGETS}
  tf2::tf2 tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_core::moveit_robot_state moveit_core::moveit_robot_trajectory
  moveit_core::moveit_trajectory_processing ${moveit_msgs_TARGETS}
  ${braco_interfaces_TARGETS})

install(TARGETS move_to_pose DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_cppcheck skips every file with cppcheck >= 2.0; make cppcheck runs cppcheck directly instead
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()

  # Pose math and the steps of a goal against a fake arm: plain C++, no ROS graph, no MoveIt
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_math test/test_pose_math.cpp)
  target_include_directories(test_pose_math PRIVATE include)
  ament_add_gtest(test_mission test/test_mission.cpp)
  target_link_libraries(test_mission braco_mission_core)
  ament_add_gtest(test_cartesian test/test_cartesian.cpp)
  target_link_libraries(test_cartesian braco_mission_core)
endif()

ament_package()
