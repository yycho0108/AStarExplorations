cmake_minimum_required(VERSION 3.8)
project(bglpy-swig)

# Find SWIG.
find_package(SWIG REQUIRED)
include(UseSWIG)

find_package(Python3 COMPONENTS Development REQUIRED)

# Set couple of package parameters.
set(SWIG_SOURCE_FILE_EXTENSIONS ".i" ".swg")
set(LIBRARY_NAME bglpy)
list(APPEND SWIG_INCLUDE_DIRECTORIES "../include")

# TODO(yycho0108): multiple sources support
set_property(SOURCE bglpy.i PROPERTY CPLUSPLUS ON)
set_property(SOURCE bglpy.i PROPERTY SWIG_MODULE_NAME bglpy)


swig_add_module(bglpy python bglpy.i)
swig_link_libraries(${LIBRARY_NAME} ${Python3_LIBRARIES} bglpy)

target_include_directories(${SWIG_MODULE_${LIBRARY_NAME}_REAL_NAME}
    PRIVATE ${Python3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/..)

set_target_properties(${SWIG_MODULE_${LIBRARY_NAME}_REAL_NAME} PROPERTIES
    INSTALL_RPATH   "$ORIGIN")

# .java -> .jar
#FILE(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/classes")
#ADD_CUSTOM_COMMAND(TARGET ${LIBRARY_NAME} POST_BUILD
#    COMMAND "${Java_JAVAC_EXECUTABLE}" -d classes *.java
#    COMMAND "${Java_JAR_EXECUTABLE}" -cfM ${CMAKE_CURRENT_SOURCE_DIR}/../../App/Android/app/libs/${LIBRARY_NAME}.jar -C classes .
#    )
#
