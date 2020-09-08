cmake_minimum_required(VERSION 3.2)

project(ORCAStar)

set(FullOutput OFF)
set(FullLog OFF)
set(PAREnvLog OFF)


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

if(PAREnvLog)
    add_compile_definitions(PAR_LOG=true)
else(PAREnvLog)
    add_compile_definitions(PAR_LOG=false)
endif(PAREnvLog)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -w -O0" )


add_subdirectory(tinyxml)
#add_subdirectory(googletest)
add_subdirectory(Src)
#add_subdirectory(Tests)
add_subdirectory(Experiments)

