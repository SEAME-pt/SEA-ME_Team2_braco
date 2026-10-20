cmake_minimum_required(VERSION 3.10)
project(braco_mission)

find_package(ament_cmake REQUIRED)

# Placeholder: the mission nodes arrive with issue #43.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
