cmake_minimum_required(VERSION 3.10)
project(braco_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mycobot_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(braco_interfaces REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# Container entrypoint (launch + web page)
install(PROGRAMS scripts/arm_routine.sh DESTINATION lib/${PROJECT_NAME})

# zero and park are run by hand (make zero / make park): C++ clients of arm_driver
add_library(arm_moves STATIC src/arm_moves.cpp)
target_include_directories(arm_moves PUBLIC include)
target_link_libraries(arm_moves PUBLIC
  mycobot_cpp::mycobot_cpp rclcpp::rclcpp
  ${sensor_msgs_TARGETS} ${std_srvs_TARGETS} ${braco_interfaces_TARGETS})

foreach(program zero park)
  add_executable(${program} src/${program}.cpp)
  target_link_libraries(${program} arm_moves)
  install(TARGETS ${program} DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
