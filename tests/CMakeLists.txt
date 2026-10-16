cmake_minimum_required(VERSION 3.16)
project(mycobot_tests CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

find_package(Threads REQUIRED)

add_executable(hello_world hello_world.cpp)
target_include_directories(hello_world PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hello_world PRIVATE Threads::Threads)
