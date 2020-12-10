############################################################
# CMakeLists for Pybind11.
#
# Copyright 2018. All Rights Reserved.
#
# Created: September 7, 2018
# Authors: Toki Migimatsu
############################################################

message(STATUS "Configuring Python wrapper for franka_panda library.")

project(franka_panda_python)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/frankapanda)

ctrl_utils_add_subdirectory(pybind11)

pybind11_add_module(frankapanda franka_panda.cc)

target_link_libraries(frankapanda
  PUBLIC
    franka_panda::franka_panda
    pybind11::pybind11
)
