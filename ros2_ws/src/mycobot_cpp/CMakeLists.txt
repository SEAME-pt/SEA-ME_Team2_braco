# CMakeLists.txt - builds the shared classes as the ROS 2 package `mycobot_cpp`
# (colcon finds it through package.xml). The Makefile next to it still builds
# and is the way to get the stand-alone programs; this file installs only the
# library they share.
#
# Other packages use it with:
#   find_package(mycobot_cpp REQUIRED)
#   target_link_libraries(my_node mycobot_cpp::mycobot_cpp)

cmake_minimum_required(VERSION 3.10)
project(mycobot_cpp)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(mycobot_cpp
  srcs/mycobot.cpp
  srcs/joystick.cpp
  srcs/display.cpp
  srcs/lcd_display.cpp
)
target_compile_options(mycobot_cpp PRIVATE -Wall -Wextra)
target_include_directories(mycobot_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/includes>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# The stand-alone programs of the Makefile, built (not installed) here too, so CI compiles them and
# make tidy sees them in compile_commands.json. Keep the list in step with PROGRAMS in the Makefile.
foreach(program park zero joystick_all_joints joystick_display pid_tool temps_lcd)
  add_executable(${program} srcs/${program}.cpp)
  target_compile_options(${program} PRIVATE -Wall -Wextra)
  target_link_libraries(${program} mycobot_cpp)
endforeach()

install(DIRECTORY includes/ DESTINATION include/${PROJECT_NAME})
install(TARGETS mycobot_cpp EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  # No arm needed: test_mycobot opens no port, test_serial talks to a pseudo-terminal. No ament
  # linters: the package is formatted with clang-format (make format-check) instead of uncrustify.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mycobot test/test_mycobot.cpp)
  target_link_libraries(test_mycobot mycobot_cpp)
  ament_add_gtest(test_serial test/test_serial.cpp TIMEOUT 120)
  target_link_libraries(test_serial mycobot_cpp util)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
