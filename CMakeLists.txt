cmake_minimum_required(VERSION 3.2)

project(ORCAStar)

set(FullOutput OFF)
set(FullLog ON)
set(MAPFEnvLog OFF)


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

if(MAPFEnvLog)
    add_compile_definitions(PAR_LOG=true)
else(MAPFEnvLog)
    add_compile_definitions(PAR_LOG=false)
endif(MAPFEnvLog)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -w -O0" )


add_subdirectory(tinyxml)
add_subdirectory(Src)
add_subdirectory(Experiments)

