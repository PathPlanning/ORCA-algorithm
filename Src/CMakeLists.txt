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
                    PushAndRotate.h
                    SubMap.h
                    PARActorSet.h
                    PARActor.h
                    ISearch.h
                    AStar.h
                    Constraints.h
                    ConflictAvoidanceTable.h)

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
                    PushAndRotate.cpp
                    SubMap.cpp
                    PARActorSet.cpp
                    PARActor.cpp
                    ISearch.cpp
                    AStar.cpp
                    Constraints.cpp
                    ConflictAvoidanceTable.cpp)

add_library(ORCAStar ${ORCAStar_HEADERS} ${ORCAStar_SOURCES})