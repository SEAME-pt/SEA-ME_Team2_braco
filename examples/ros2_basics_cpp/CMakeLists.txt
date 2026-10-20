cmake_minimum_required(VERSION 3.10)
project(ros2_basics_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_basics_interfaces REQUIRED)

add_executable(talker src/talker.cpp)
target_link_libraries(talker rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(listener src/listener.cpp)
target_link_libraries(listener rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server rclcpp::rclcpp ${ros2_basics_interfaces_TARGETS})

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client rclcpp::rclcpp ${ros2_basics_interfaces_TARGETS})

add_executable(managed_talker src/managed_talker.cpp)
target_link_libraries(managed_talker
  rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle ${std_msgs_TARGETS})

install(TARGETS
  talker listener add_two_ints_server add_two_ints_client managed_talker
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
