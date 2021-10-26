project(ecu CXX)

cmake_minimum_required(VERSION 3.17)

include_directories(include)
include_directories(.)

add_subdirectory(tests)
add_subdirectory(e591)

add_executable(e591 e591.cpp)

