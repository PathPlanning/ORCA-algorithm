cmake_minimum_required(VERSION 3.17)
set(CMAKE_CXX_STANDARD 17)

project(multi-agent-navigation)

set(FULL_OUTPUT OFF)
set(FULL_LOG ON)
set(MAPF_LOG OFF)


if(FULL_OUTPUT)
    add_compile_definitions(FULL_OUTPUT=true)
else(FULL_OUTPUT)
    add_compile_definitions(FULL_OUTPUT=false)
endif(FULL_OUTPUT)

if(FULL_LOG)
    add_compile_definitions(FULL_LOG=true)
else(FULL_LOG)
    add_compile_definitions(FULL_LOG=false)
endif(FULL_LOG)

if(MAPF_LOG)
    add_compile_definitions(PAR_LOG=true)
else(MAPF_LOG)
    add_compile_definitions(PAR_LOG=false)
endif(MAPF_LOG)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -w -O0" )

add_subdirectory(external/tinyxml2)


SET(MA_NAV_LIB_SOURCES
        ${PROJECT_SOURCE_DIR}/src/Agent.cpp
        ${PROJECT_SOURCE_DIR}/src/XMLLogger.cpp
        ${PROJECT_SOURCE_DIR}/src/Mission.cpp
        ${PROJECT_SOURCE_DIR}/src/Map.cpp
        ${PROJECT_SOURCE_DIR}/src/XMLReader.cpp
        ${PROJECT_SOURCE_DIR}/src/ThetaStar.cpp
        ${PROJECT_SOURCE_DIR}/src/Geom.cpp
        ${PROJECT_SOURCE_DIR}/src/EnvironmentOptions.cpp
        ${PROJECT_SOURCE_DIR}/src/DirectPlanner.cpp
        ${PROJECT_SOURCE_DIR}/src/ORCAAgent.cpp
        ${PROJECT_SOURCE_DIR}/src/ORCADDAgent.cpp
        ${PROJECT_SOURCE_DIR}/src/ORCAAgentWithPAR.cpp
        ${PROJECT_SOURCE_DIR}/src/SubMap.cpp
        ${PROJECT_SOURCE_DIR}/src/MAPFInstancesLogger.cpp
        ${PROJECT_SOURCE_DIR}/src/ORCAAgentWithECBS.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/PushAndRotate.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/MAPFActorSet.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/MAPFActor.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/ISearch.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/AStar.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/SearchQueue.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/Constraints.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/ConflictAvoidanceTable.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/ConflictBasedSearch.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/ConflictSet.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/MDD.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/FocalSearch.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/SCIPP.cpp
        ${PROJECT_SOURCE_DIR}/src/mapf/SIPP.cpp
        ${PROJECT_SOURCE_DIR}/src/ORCAAgentWithPARAndECBS.cpp
        ${PROJECT_SOURCE_DIR}/src/ORCAAgentWithReturning.cpp
        ${PROJECT_SOURCE_DIR}/src/agent_goal_exchange.cpp)



add_library(ma_navigation_lib STATIC ${MA_NAV_LIB_SOURCES})

target_include_directories(ma_navigation_lib
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        )
target_link_libraries(ma_navigation_lib tinyxml2)

add_executable(single_test src/experiments/SingleTest.cpp)
add_executable(series_test src/experiments/SeriesTest.cpp)

target_link_libraries(single_test ma_navigation_lib tinyxml2)
target_link_libraries(series_test ma_navigation_lib tinyxml2)