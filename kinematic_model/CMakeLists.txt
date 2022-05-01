cmake_minimum_required(VERSION 3.2)

# Set up project.
project(kinematic_model
        VERSION 1.0
        DESCRIPTION "A C++ library for estimating the state of a kinematic system.")

# Add source files.
file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

# Create library.
add_library(kinematic_model STATIC ${sources})

# Link library to dependencies.
target_link_libraries(kinematic_model
    transform
    kalman_filter)

# Set include directories for internal/external use.
target_include_directories(kinematic_model PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)