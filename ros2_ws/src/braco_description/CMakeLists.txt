cmake_minimum_required(VERSION 3.10)
project(braco_description)

find_package(ament_cmake REQUIRED)

# Placeholder: the URDF, meshes and RViz config arrive with issue #25 (install them from here).

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
