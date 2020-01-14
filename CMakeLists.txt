cmake_minimum_required(VERSION 3.2)

project(ORCA)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O0" )

enable_testing()

add_subdirectory(tinyxml)
add_subdirectory(Src)
add_subdirectory(Tests)

