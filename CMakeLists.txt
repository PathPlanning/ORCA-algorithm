cmake_minimum_required(VERSION 3.2)

project(ORCA)

set(FullOutput OFF)
set(FullLog OFF)


if(FullOutput)
    add_compile_definitions(FULL_OUTPUT=true)
else(FullOutput)
    add_compile_definitions(FULL_OUTPUT=false)
endif(FullOutput)

if(FullLog)
    add_compile_definitions(FULL_LOG=true)
else(FullLog)
    add_compile_definitions(FULL_LOG=false)
endif(FullLog)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -O0" )


enable_testing()

add_subdirectory(tinyxml)
add_subdirectory(Src)
add_subdirectory(Tests)

