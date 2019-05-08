cmake_minimum_required(VERSION 3.2)
project(ORCA)

set(CMAKE_CXX_STANDARD 11)

add_executable(ORCA main.cpp Agent.cpp Agent.h Point.cpp Point.h Line.h tinyxml2.h tinyxml2.cpp XmlLogger.cpp XmlLogger.h Mission.cpp Mission.h)