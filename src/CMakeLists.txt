set(MAPF_HEADERS    ../include/MAPF/AStar.h
                    ../include/MAPF/CBSNode.h
                    ../include/MAPF/Conflict.h
                    ../include/MAPF/ConflictAvoidanceTable.h
                    ../include/MAPF/ConflictBasedSearch.h
                    ../include/MAPF/ConflictSet.h
                    ../include/MAPF/Constraints.h
                    ../include/MAPF/FocalSearch.h
                    ../include/MAPF/FSNode.h
                    ../include/MAPF/ISearch.h
                    ../include/MAPF/MAPFActor.h
                    ../include/MAPF/MAPFActorSet.h
                    ../include/MAPF/MAPFConfig.h
                    ../include/MAPF/MAPFSearchInterface.h
                    ../include/MAPF/MAPFSearchResult.h
                    ../include/MAPF/MDD.h
                    ../include/MAPF/PushAndRotate.h
                    ../include/MAPF/SCIPP.h
                    ../include/MAPF/SCIPPNode.h
                    ../include/MAPF/SearchQueue.h
                    ../include/MAPF/SIPP.h
                    ../include/MAPF/SIPPNode.h)


set(MAPF_SOURCES    MAPF/AStar.cpp
                    MAPF/ConflictAvoidanceTable.cpp
                    MAPF/ConflictBasedSearch.cpp
                    MAPF/ConflictSet.cpp
                    MAPF/Constraints.cpp
                    MAPF/FocalSearch.cpp
                    MAPF/ISearch.cpp
                    MAPF/MAPFActor.cpp
                    MAPF/MAPFActorSet.cpp
                    MAPF/MDD.cpp
                    MAPF/PushAndRotate.cpp
                    MAPF/SCIPP.cpp
                    MAPF/SearchQueue.cpp
                    MAPF/SIPP.cpp)


set(ORCA_HEADERS    ../include/Agent.h
                    ../include/Map.h
                    ../include/Const.h
                    ../include/PathPlanner.h
                    ../include/EnvironmentOptions.h
                    ../include/Geom.h
                    ../include/ThetaStar.h
                    ../include/LineOfSight.h
                    ../include/DirectPlanner.h
                    ../include/ORCAAgent.h
                    ../include/ORCADDAgent.h
                    ../include/ORCAAgentWithPAR.h
                    ../include/SubMap.h
                    ../include/ORCAAgentWithECBS.h
                    ../include/ORCAAgentWithPARAndECBS.h
                    ../include/ORCAAgentWithReturning.h)

set(ORCA_SOURCES    Agent.cpp
                    Map.cpp
                    ThetaStar.cpp
                    Geom.cpp
                    EnvironmentOptions.cpp
                    DirectPlanner.cpp
                    ORCAAgent.cpp
                    ORCADDAgent.cpp
                    ORCAAgentWithPAR.cpp
                    SubMap.cpp
                    ORCAAgentWithECBS.cpp
                    ORCAAgentWithPARAndECBS.cpp
                    ORCAAgentWithReturning.cpp)

add_library(ORCAStar ${ORCA_HEADERS} ${ORCA_SOURCES} ${MAPF_HEADERS} ${MAPF_SOURCES})
target_include_directories(ORCAStar PUBLIC ../include)
target_link_libraries(ORCAStar TINYXML)



