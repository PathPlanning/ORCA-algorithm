cmake_minimum_required(VERSION 3.12)
project(ORCA)

set(CMAKE_CXX_STANDARD 11)

add_executable(ORCA main.cpp Agent.cpp Agent.h Point.cpp Point.h)