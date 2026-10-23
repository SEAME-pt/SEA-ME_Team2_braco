cmake_minimum_required(VERSION 3.10)
project(braco_camera)

find_package(ament_cmake REQUIRED)

# A Python node, installed like a program: ros2 run braco_camera camera
install(PROGRAMS scripts/camera.py DESTINATION lib/${PROJECT_NAME} RENAME camera)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
