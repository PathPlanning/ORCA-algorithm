enable_testing()
include(GoogleTest)
include_directories(../Src)



set(BINARY ORCAStarTests)

file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false *.h *.cpp)

set(SOURCES ${TEST_SOURCES})

add_executable(${BINARY} ${TEST_SOURCES})

add_test(ORCAStarTests ${BINARY} ORCAStarTests ${BINARY})

target_link_libraries(${BINARY} PUBLIC ORCAStar TINYXML gtest )


