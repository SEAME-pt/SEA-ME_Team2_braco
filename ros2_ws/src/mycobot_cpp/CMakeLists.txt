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
set(TOOLS park zero joystick_all_joints joystick_display pid_tool temps_lcd)
foreach(program ${TOOLS})
  add_executable(${program} srcs/${program}.cpp)
  target_compile_options(${program} PRIVATE -Wall -Wextra)
  target_link_libraries(${program} mycobot_cpp)
endforeach()

install(DIRECTORY includes/ DESTINATION include/${PROJECT_NAME})
# Test code, not part of the library: the fake arm on a pseudo-terminal, for other packages' tests.
install(FILES test/pty_arm.hpp DESTINATION include/${PROJECT_NAME})
install(TARGETS mycobot_cpp EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  # No arm needed: test_mycobot opens no port, test_serial and test_tools talk to a pseudo-terminal
  # (test/pty_arm.hpp). No ament linters: the package is formatted with clang-format
  # (make format-check) instead of uncrustify.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mycobot test/test_mycobot.cpp)
  target_link_libraries(test_mycobot mycobot_cpp)
  ament_add_gtest(test_serial test/test_serial.cpp TIMEOUT 120)
  target_link_libraries(test_serial mycobot_cpp util)
  # Runs the stand-alone programs themselves (built next to each other, in TOOLS_DIR).
  ament_add_gtest(test_tools test/test_tools.cpp TIMEOUT 120)
  target_link_libraries(test_tools mycobot_cpp util)
  target_compile_definitions(test_tools PRIVATE TOOLS_DIR="$<TARGET_FILE_DIR:park>")
  add_dependencies(test_tools ${TOOLS})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
