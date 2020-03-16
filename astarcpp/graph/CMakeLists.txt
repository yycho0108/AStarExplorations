cmake_minimum_required(VERSION 3.14)
project(bglpy)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(Boost_USE_STATIC_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(fmt REQUIRED)


# Actual lib
add_library(bglpy src/graph.cpp include/graph.hpp src/rrt.cpp include/rrt.hpp include/types.hpp include/sdf.hpp src/sdf.cpp src/types.cpp)
target_include_directories(bglpy PUBLIC include/ ${Eigen_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(bglpy "" Eigen3::Eigen fmt ${Boost_LIBRARIES})

# Sample script
add_executable(main src/main.cpp)
target_include_directories(main PUBLIC include/ ${Eigen_INCLUDE_DIRS})
target_link_libraries(main "" Eigen3::Eigen fmt bglpy)

add_subdirectory(swig)
