############################################################
# CMakeLists for the example Frank Panda Opspace Controller
#
# Copyright 2019. All Rights Reserved.
# Stanford IPRL
#
# Created: January 09, 2019
# Authors: Toki Migimatsu
############################################################

# Require 3.6 to support pkg_check_modules IMPORTED_TARGET
cmake_minimum_required(VERSION 3.6)

# Define project
project(franka_panda_opspace VERSION 1.0.0 LANGUAGES CXX)

# Set CMake flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON) # TODO: Remove
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

# Define directories
set(FRANKA_OPSPACE_BIN franka_panda_opspace)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(FRANKA_OPSPACE_SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# Create Franka Panda driver
add_executable(${FRANKA_OPSPACE_BIN})

# Set compiler options
target_compile_options(${FRANKA_OPSPACE_BIN} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wall>
    $<$<CONFIG:RELEASE>:-O3>
)

find_package(SpatialDyn 1.0.0 REQUIRED)
find_package(FrankaPanda 1.0.0 REQUIRED)

find_package(yaml-cpp 0.6.2 REQUIRED)
add_library(yaml-cpp::yaml-cpp INTERFACE IMPORTED)
set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
    INTERFACE_LINK_LIBRARIES yaml-cpp
    INTERFACE_INCLUDE_DIRECTORIES ${YAML_CPP_INCLUDE_DIR})

target_link_libraries(${FRANKA_OPSPACE_BIN}
    PRIVATE
        SpatialDyn::SpatialDyn
        FrankaPanda::FrankaPanda
        yaml-cpp::yaml-cpp
)

# Attach sources
target_sources(${FRANKA_OPSPACE_BIN}
    PRIVATE
        ${FRANKA_OPSPACE_SRC_DIR}/main.cc
)

