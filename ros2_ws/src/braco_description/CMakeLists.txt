cmake_minimum_required(VERSION 3.10)
project(braco_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config DESTINATION share/${PROJECT_NAME})
# Code driving the Gazebo arm: ros2 run braco_description move_arm_example.py / park.py
# (park.py imports MoveArm from move_arm_example.py: keep them in the same folder)
install(PROGRAMS scripts/move_arm_example.py scripts/park.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py)
endif()

ament_package()
