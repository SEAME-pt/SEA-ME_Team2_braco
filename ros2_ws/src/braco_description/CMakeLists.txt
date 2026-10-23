cmake_minimum_required(VERSION 3.10)
project(braco_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch config DESTINATION share/${PROJECT_NAME})
# Moves in Gazebo, then (--real) on the real arm: ros2 run braco_description park.py [--real]
# The scripts import braco_arm.py from their own folder: install it next to them.
install(PROGRAMS scripts/move_arm_example.py scripts/park.py scripts/sim_to_real.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/braco_arm.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf test/test_urdf.py)
endif()

ament_package()
