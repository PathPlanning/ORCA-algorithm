include_directories(../Src)

add_executable(Single SingleTest.cpp)
add_executable(Series SeriesTest.cpp)

target_link_libraries(Single ORCA TINYXML)
target_link_libraries(Series ORCA TINYXML)

add_test(Single Single)
add_test(Series Series)