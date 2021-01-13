############################################################
# CMakeLists for Pybind11.
#
# Copyright 2018. All Rights Reserved.
#
# Created: September 7, 2018
# Authors: Toki Migimatsu
############################################################

message(STATUS "Configuring Python wrapper for ${PROJECT_NAME}")

set(PYTHON_LIB_NAME frankapanda)
set(PYTHON_LIB_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${PYTHON_LIB_NAME}")

# Create the library.
ctrl_utils_add_subdirectory(pybind11)
pybind11_add_module(frankapanda franka_panda.cc)

# Set the library output directory.
set_target_properties(${PYTHON_LIB_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PYTHON_LIB_DIR}"
)

# Link library dependencies.
target_link_libraries(frankapanda
  PUBLIC
    franka_panda::franka_panda
    pybind11::pybind11
)

# Install python scripts.
set(PYTHON_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/__init__.py"
)
add_custom_command(
    TARGET ${PYTHON_LIB_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PYTHON_SOURCES} "${PYTHON_LIB_DIR}"
)
