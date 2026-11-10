cmake_minimum_required(VERSION 3.10)
project(braco_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
# MoveItConfigsBuilder (and the Setup Assistant, to edit this package) read it from the share folder
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
# The table tops into move_group's planning scene, run by launch/sim.launch.py
install(PROGRAMS scripts/lab_bench_scene.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_moveit_config test/test_moveit_config.py)
endif()

ament_package()
