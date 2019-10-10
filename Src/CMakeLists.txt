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
                    DirectPlanner.h)

SET(ORCA_SOURCES    Agent.cpp
                    XMLLogger.cpp
                    Mission.cpp
                    Map.cpp
                    XMLReader.cpp
                    ThetaStar.cpp
                    Geom.cpp
                    EnvironmentOptions.cpp
                    DirectPlanner.cpp)

add_library(ORCA ${ORCA_HEADERS} ${ORCA_SOURCES})