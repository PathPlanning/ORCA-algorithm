
set(SIM_HEADERS Mission.h
        XMLLogger.h
        Logger.h
        Reader.h
        XMLReader.h
        Summary.h
        MAPFInstancesLogger.h
        xml_tags.h)

set(SIM_SOURCES XMLLogger.cpp
                Mission.cpp
                XMLReader.cpp
                MAPFInstancesLogger.cpp)

add_executable(single single_experiment.cpp ${SIM_HEADERS} ${SIM_SOURCES})
add_executable(series series_experiment.cpp ${SIM_HEADERS} ${SIM_SOURCES})


target_link_libraries(single ORCAStar TINYXML)
target_include_directories(single PUBLIC ../include)
target_include_directories(single PUBLIC ../libs/tinyxml)



target_link_libraries(series ORCAStar TINYXML)

