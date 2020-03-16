cmake_minimum_required(VERSION 3.8)
project(bglpy-swig)

# Find SWIG.
find_package(SWIG REQUIRED)
include(UseSWIG)

find_package(Python3 COMPONENTS Development REQUIRED)

# Set couple of package parameters.
set(LIBRARY_NAME bglpy)

# Configure SWIG .
set(SWIG_SOURCE_FILE_EXTENSIONS ".i")
set(SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON)
list(APPEND SWIG_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/../include/")
message("INCLUDE ${SWIG_INCLUDE_DIRECTORIES}")

# TODO(yycho0108): multiple sources support
set_property(SOURCE bglpy.i PROPERTY SWIG_MODULE_NAME bglpy)
set_property(SOURCE bglpy.i PROPERTY CPLUSPLUS ON)

# All those include stuff
set_property(SOURCE bglpy.i PROPERTY INCLUDE_DIRECTORIES "${SWIG_INCLUDE_DIRECTORIES}")

swig_add_library(bglpy
    TYPE SHARED
    LANGUAGE python
    SOURCES bglpy.i
    )

swig_link_libraries(${LIBRARY_NAME}
    ${Python3_LIBRARIES}
    bglpy)

set_property(TARGET bglpy PROPERTY SWIG_USE_LIBRARY_INCLUDE_DIRECTORIES TRUE)

target_include_directories(${SWIG_MODULE_${LIBRARY_NAME}_REAL_NAME}
    PRIVATE ${Python3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../include)

set_target_properties(${SWIG_MODULE_${LIBRARY_NAME}_REAL_NAME} PROPERTIES
    INSTALL_RPATH   "$ORIGIN")

# .java -> .jar
#FILE(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/classes")
#ADD_CUSTOM_COMMAND(TARGET ${LIBRARY_NAME} POST_BUILD
#    COMMAND "${Java_JAVAC_EXECUTABLE}" -d classes *.java
#    COMMAND "${Java_JAR_EXECUTABLE}" -cfM ${CMAKE_CURRENT_SOURCE_DIR}/../../App/Android/app/libs/${LIBRARY_NAME}.jar -C classes .
#    )
#
