SET(ORCA_HEADERS    Agent.h
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
                    SubMap.h)

SET(ORCA_SOURCES    Agent.cpp
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
                    SubMap.cpp PARActorSet.cpp PARActorSet.h PARActor.cpp PARActor.h ISearch.cpp ISearch.h AStar.cpp AStar.h Constraints.cpp Constraints.h ConflictAvoidanceTable.cpp ConflictAvoidanceTable.h)

add_library(ORCA ${ORCA_HEADERS} ${ORCA_SOURCES})