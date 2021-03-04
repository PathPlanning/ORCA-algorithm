include_directories(../Src)

add_executable(Single SingleTest.cpp)
add_executable(Series SeriesTest.cpp)

target_link_libraries(Single ORCAStar TINYXML)
target_link_libraries(Series ORCAStar TINYXML)

