cmake_minimum_required(VERSION 3.10)
project(braco_moveit_config)

find_package(ament_cmake REQUIRED)

# Placeholder: the MoveIt 2 configuration arrives with issue #40 (install config/ from here).

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
