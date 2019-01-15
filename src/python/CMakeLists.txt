############################################################
# CMakeLists for Pybind11
#
# Copyright 2018. All Rights Reserved.
#
# Created: September 7, 2018
# Authors: Toki Migimatsu
############################################################

#############
# Pybind11 #
#############

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/frankapanda)

find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
    download_package(pybind11)
    set(PYBIND11_LIB_DIR ${FRANKA_PANDA_EXT_DIR}/pybind11.git)
    add_subdirectory(${PYBIND11_LIB_DIR} ${CMAKE_CURRENT_BINARY_DIR}/pybind11)
endif()

pybind11_add_module(frankapanda franka_panda.cc)
target_link_libraries(frankapanda
    PUBLIC
        pybind11::pybind11
        franka_panda::franka_panda)
