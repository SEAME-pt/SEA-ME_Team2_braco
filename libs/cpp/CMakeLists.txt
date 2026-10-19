# CMakeLists.txt - builds the shared classes as the ROS 2 package `mycobot_cpp`
# (colcon finds it through package.xml). The Makefile next to it still builds
# the stand-alone programs; this file only builds the library they share.
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

install(DIRECTORY includes/ DESTINATION include/${PROJECT_NAME})
install(TARGETS mycobot_cpp EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
