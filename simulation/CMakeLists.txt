
set(SIM_HEADERS include/Mission.h
                include/XMLLogger.h
                include/Logger.h
                include/Reader.h
                include/XMLReader.h
                include/Summary.h
                include/MAPFInstancesLogger.h
                include/xml_tags.h)

set(SIM_SOURCES XMLLogger.cpp
                Mission.cpp
                XMLReader.cpp
                MAPFInstancesLogger.cpp)

add_executable(single single_experiment.cpp ${SIM_HEADERS} ${SIM_SOURCES})
add_executable(series series_experiment.cpp ${SIM_HEADERS} ${SIM_SOURCES})

target_link_libraries(single PRIVATE ORCAStar TINYXML)
target_link_libraries(series PRIVATE ORCAStar TINYXML)

