cmake_minimum_required(VERSION 3.4)
project(path_finding)
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(googletest)

add_executable(path_finding src/main.cpp src/path_finding.cpp)
target_include_directories(path_finding PUBLIC include)

add_executable(path_finding_test tests/test.cpp src/path_finding.cpp)
target_include_directories(path_finding_test PUBLIC googletest/googletest/include include)
target_link_libraries(path_finding_test gtest gtest_main)

enable_testing()
add_test(NAME path_finding_test COMMAND path_finding_test)