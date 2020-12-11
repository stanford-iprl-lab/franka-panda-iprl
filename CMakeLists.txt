############################################################
# CMakeLists for the Franka Panda driver.
#
# Copyright 2018. All Rights Reserved.
# Stanford IPRL
#
# Created: December 19, 2018
# Authors: Toki Migimatsu
############################################################

cmake_minimum_required(VERSION 3.11)

# Define project.
project(franka_panda
    VERSION 1.2.0
    DESCRIPTION "IPRL Franka Panda control library"
    LANGUAGES CXX)
string(TOUPPER ${PROJECT_NAME} LIB_CMAKE_NAME)

# Include custom CMake utilities.
include(cmake/utils.cmake)

# Detect whether this project is built by itself or included as a subdirectory.
lib_dependent_option(${LIB_CMAKE_NAME}_MAIN_BUILD "Build ${PROJECT_NAME} as a main project."
    "CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME")

# Define CMake options.
lib_option(BUILD_EXAMPLES "Build examples." ON)
lib_option(BUILD_PYTHON "Build Python wrapper." ON)
lib_option(CLANG_TIDY "Perform clang-tidy checks." OFF)

# Set default build type to release.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Set output directories.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# Standardize install directories.
include(GNUInstallDirs)

if(${LIB_CMAKE_NAME}_BUILD_PYTHON)
    option(CTRL_UTILS_BUILD_PYTHON "Build Python wrapper for ctrl_utils" ON)
    option(SPATIAL_DYN_BUILD_PYTHON "Build Python wrapper for spatial_dyn" ON)
endif()

# Build the library.
add_subdirectory(src)

# Build a Python wrapper for the library.
if(${LIB_CMAKE_NAME}_BUILD_PYTHON)
    add_subdirectory(python)
endif()

# Build examples.
if(${LIB_CMAKE_NAME}_BUILD_EXAMPLES)
    add_subdirectory(apps)
endif()
