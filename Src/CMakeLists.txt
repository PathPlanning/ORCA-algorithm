SET(ORCAStar_HEADERS    Agent.h
                    Mission.h
                    XMLLogger.h
                    Map.h
                    Const.h
                    PathPlanner.h
                    EnvironmentOptions.h
                    Logger.h
                    Reader.h
                    XMLReader.h
                    Geom.h
                    Summary.h
                    ThetaStar.h
                    LineOfSight.h
                    DirectPlanner.h
                    ORCAAgent.h
                    ORCADDAgent.h
                    ORCAAgentWithPAR.h
                    SubMap.h
                    MAPFInstancesLogger.h
                    ORCAAgentWithECBS.h
                    MAPF/PAR/PushAndRotate.h
                    MAPF/MAPFActorSet.h
                    MAPF/MAPFActor.h
                    MAPF/ISearch.h
                    MAPF/AStar.h
                    MAPF/MAPFSearchResult.h
                    MAPF/MAPFSearchInterface.h
                    MAPF/MAPFConfig.h
                    MAPF/SearchQueue.h
                    MAPF/ECBS/Constraints.h
                    MAPF/ECBS/ConflictAvoidanceTable.h
                    MAPF/ECBS/Conflict.h
                    MAPF/ECBS/CBSNode.h
                    MAPF/ECBS/ConflictBasedSearch.h
                    MAPF/ECBS/ConflictSet.h
                    MAPF/ECBS/MDD.h MAPF/SIPPNode.h MAPF/SCIPPNode.h)

SET(ORCAStar_SOURCES    Agent.cpp
                    XMLLogger.cpp
                    Mission.cpp
                    Map.cpp
                    XMLReader.cpp
                    ThetaStar.cpp
                    Geom.cpp
                    EnvironmentOptions.cpp
                    DirectPlanner.cpp
                    ORCAAgent.cpp
                    ORCADDAgent.cpp
                    ORCAAgentWithPAR.cpp
                    SubMap.cpp
                    MAPFInstancesLogger.cpp
                    ORCAAgentWithECBS.cpp
                    MAPF/PAR/PushAndRotate.cpp
                    MAPF/MAPFActorSet.cpp
                    MAPF/MAPFActor.cpp
                    MAPF/ISearch.cpp
                    MAPF/AStar.cpp
                    MAPF/SearchQueue.cpp
                    MAPF/ECBS/Constraints.cpp
                    MAPF/ECBS/ConflictAvoidanceTable.cpp
                    MAPF/ECBS/ConflictBasedSearch.cpp
                    MAPF/ECBS/ConflictSet.cpp
                    MAPF/ECBS/MDD.cpp MAPF/FocalSearch.cpp MAPF/FocalSearch.h MAPF/FSNode.h MAPF/SCIPP.cpp MAPF/SCIPP.h MAPF/SIPP.cpp MAPF/SIPP.h ORCAAgenWithPARAndECBS.cpp ORCAAgenWithPARAndECBS.h)

add_library(ORCAStar ${ORCAStar_HEADERS} ${ORCAStar_SOURCES})