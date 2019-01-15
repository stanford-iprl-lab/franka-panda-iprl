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
set(FRANKA_PANDA_EXT_DIR ${PROJECT_SOURCE_DIR}/../../external)

# Create Franka Panda driver
add_executable(${FRANKA_OPSPACE_BIN})

# Set compiler options
target_compile_options(${FRANKA_OPSPACE_BIN} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wall>
    $<$<CONFIG:RELEASE>:-O3>
)

find_package(spatial_dyn 1.0.0 REQUIRED)
find_package(franka_panda 1.0.0 REQUIRED)
find_package(ctrl_utils 1.0.0 REQUIRED)

target_link_libraries(${FRANKA_OPSPACE_BIN}
    PRIVATE
        spatial_dyn::spatial_dyn
        franka_panda::franka_panda
        ctrl_utils::ctrl_utils
)

# Attach sources
target_sources(${FRANKA_OPSPACE_BIN}
    PRIVATE
        ${FRANKA_OPSPACE_SRC_DIR}/main.cc
)

find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
    set(PYBIND11_LIB_DIR ${FRANKA_PANDA_EXT_DIR}/pybind11.git)
    add_subdirectory(${PYBIND11_LIB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/pybind11)
endif()

pybind11_add_module(spatialdyn_frankapanda ${FRANKA_OPSPACE_SRC_DIR}/python_bindings.cc)
target_link_libraries(spatialdyn_frankapanda
    PUBLIC
        spatial_dyn::spatial_dyn
        franka_panda::franka_panda
        pybind11::pybind11)
