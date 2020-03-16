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
set(SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/types.cpp
    ${PROJECT_SOURCE_DIR}/src/sdf.cpp
    ${PROJECT_SOURCE_DIR}/src/rrt.cpp
    ${PROJECT_SOURCE_DIR}/src/graph.cpp
    )

set(HDR_FILES
    ${PROJECT_SOURCE_DIR}/include/bglpy/types.hpp
    ${PROJECT_SOURCE_DIR}/include/bglpy/sdf.hpp
    ${PROJECT_SOURCE_DIR}/include/bglpy/rrt.hpp
    ${PROJECT_SOURCE_DIR}/include/bglpy/graph.hpp
    )


add_library(bglpy ${SRC_FILES} ${HDR_FILES})
target_include_directories(bglpy PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Eigen_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
    )
target_link_libraries(bglpy "" Eigen3::Eigen fmt ${Boost_LIBRARIES})


# Sample script
add_executable(main src/main.cpp)
target_include_directories(main PUBLIC include/ ${Eigen_INCLUDE_DIRS})
target_link_libraries(main "" Eigen3::Eigen fmt bglpy)

add_subdirectory(swig)
