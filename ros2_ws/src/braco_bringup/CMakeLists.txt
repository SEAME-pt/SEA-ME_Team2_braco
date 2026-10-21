cmake_minimum_required(VERSION 3.10)
project(braco_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# Container entrypoint (launch + web page); zero.py / park.py are run by hand (make zero / make park)
install(PROGRAMS scripts/zero.py scripts/park.py scripts/arm_routine.sh
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/arm_moves.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
