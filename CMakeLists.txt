cmake_minimum_required(VERSION 3.2)

# Set up project.
project(kinematic_model
        VERSION 1.0
        DESCRIPTION "A C++ library for estimating the state of a kinematic system.")

# Add project subdirectories.
add_subdirectory("kinematic_model")
add_subdirectory("kalman_filter")
add_subdirectory("transform")