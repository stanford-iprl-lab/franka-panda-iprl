############################################################
# CMakeLists for the Frank Panda driver
#
# Copyright 2018. All Rights Reserved.
# Stanford IPRL
#
# Created: December 19, 2018
# Authors: Toki Migimatsu
############################################################

# Require 3.6 to support pkg_check_modules IMPORTED_TARGET
cmake_minimum_required(VERSION 3.6)

# Define project
project(franka_panda VERSION 1.0.0 LANGUAGES CXX)

# Set CMake flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

# Define directories
set(FRANKA_PANDA_LIB franka_panda)
set(FRANKA_PANDA_BIN franka_panda_driver)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(FRANKA_PANDA_CMAKE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(FRANKA_PANDA_EXT_DIR ${PROJECT_SOURCE_DIR}/external)
set(FRANKA_PANDA_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(FRANKA_PANDA_SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# Create Franka Panda driver
add_executable(${FRANKA_PANDA_BIN})

# Set compiler options
target_compile_options(${FRANKA_PANDA_BIN} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wall>
    $<$<CONFIG:RELEASE>:-O3>
)

# Find dependencies
function(download_package PACKAGE_NAME)
    configure_file(${FRANKA_PANDA_CMAKE_DIR}/${PACKAGE_NAME}-CMakeLists.txt.in
        ${CMAKE_BINARY_DIR}/${PACKAGE_NAME}/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/${PACKAGE_NAME})
endfunction()

# Export for build tree
configure_file(${FRANKA_PANDA_CMAKE_DIR}/Findcpp_redis.cmake.in
    ${CMAKE_BINARY_DIR}/Findcpp_redis.cmake @ONLY)
configure_file(${FRANKA_PANDA_CMAKE_DIR}/FindEigen3.cmake.in
    ${CMAKE_BINARY_DIR}/FindEigen3.cmake @ONLY)
configure_file(${FRANKA_PANDA_CMAKE_DIR}/Findyaml-cpp.cmake.in
    ${CMAKE_BINARY_DIR}/Findyaml-cpp.cmake @ONLY)

find_package(Franka 0.5.0 QUIET)
if (NOT Franka_FOUND)
    download_package(Franka)
    find_package(Franka 0.5.0 REQUIRED PATHS ${CMAKE_BINARY_DIR}/libfranka/build)
endif()

find_package(ctrl_utils 1.0.0 QUIET)
if(NOT ctrl_utils_FOUND)
    download_package(ctrl_utils)
    find_package(ctrl_utils 1.0.0 REQUIRED)
endif()

target_link_libraries(${FRANKA_PANDA_BIN}
    PRIVATE
        ctrl_utils::ctrl_utils
        Franka::Franka
        atomic  # Needed for gcc 5.4.0
)

# Set include directories
target_include_directories(${FRANKA_PANDA_BIN}
    PRIVATE
        ${FRANKA_PANDA_SRC_DIR})

# Attach sources
target_sources(${FRANKA_PANDA_BIN}
    PRIVATE
        ${FRANKA_PANDA_SRC_DIR}/main.cc
        ${FRANKA_PANDA_SRC_DIR}/args.cc
        ${FRANKA_PANDA_SRC_DIR}/controllers.cc
)

# Create franka_panda library
add_library(${FRANKA_PANDA_LIB} SHARED)
add_library(${FRANKA_PANDA_LIB}::${FRANKA_PANDA_LIB} ALIAS ${FRANKA_PANDA_LIB})

# Set compiler options
target_compile_options(${FRANKA_PANDA_BIN} PRIVATE
    $<$<CXX_COMPILER_ID:GNU>:-Wall>
    $<$<CONFIG:RELEASE>:-O3>
)

# Link internal model library
find_library(FCI_MODELS_LIB fcimodels REQUIRED PATHS ${FRANKA_PANDA_EXT_DIR})
target_link_libraries(${FRANKA_PANDA_LIB}
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        ${FCI_MODELS_LIB}
)

# Set include directories
target_include_directories(${FRANKA_PANDA_LIB}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${FRANKA_PANDA_INCLUDE_DIR}>
    PRIVATE
        ${FRANKA_PANDA_EXT_DIR}/libfranka.git/src
)

# Attach sources
target_sources(${FRANKA_PANDA_LIB}
    PRIVATE
        ${FRANKA_PANDA_SRC_DIR}/model.cc
)

# # Use GNUInstalDirs to install ibraries into correct locations on all platforms
include(GNUInstallDirs)

set(FRANKA_PANDA_TARGETS ${FRANKA_PANDA_LIB}Targets)
set(FRANKA_PANDA_CONFIG ${FRANKA_PANDA_LIB}Config)
set(FRANKA_PANDA_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${FRANKA_PANDA_LIB})

# Install the library
install(TARGETS ${FRANKA_PANDA_LIB}
    EXPORT ${FRANKA_PANDA_TARGETS}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install the headers
install(DIRECTORY ${FRANKA_PANDA_INCLUDE_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export targets to Targets.cmake
install(EXPORT ${FRANKA_PANDA_TARGETS}
    DESTINATION ${FRANKA_PANDA_CONFIG_INSTALL_DIR}
    NAMESPACE ${FRANKA_PANDA_LIB}::
    FILE ${FRANKA_PANDA_TARGETS}.cmake
)

# Create ConfigVersion.cmake
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_BINARY_DIR}/${FRANKA_PANDA_CONFIG}Version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Create Config.cmake
configure_package_config_file(${FRANKA_PANDA_CMAKE_DIR}/${FRANKA_PANDA_CONFIG}.cmake.in
    ${CMAKE_BINARY_DIR}/${FRANKA_PANDA_CONFIG}.cmake
    INSTALL_DESTINATION ${FRANKA_PANDA_CONFIG_INSTALL_DIR})

# Install config script files
install(FILES
    ${CMAKE_BINARY_DIR}/${FRANKA_PANDA_CONFIG}.cmake
    ${CMAKE_BINARY_DIR}/${FRANKA_PANDA_CONFIG}Version.cmake
    DESTINATION ${FRANKA_PANDA_CONFIG_INSTALL_DIR}
)

# Build python wrapper
add_subdirectory(${FRANKA_PANDA_SRC_DIR}/python)

export(TARGETS ${FRANKA_PANDA_LIB}
    NAMESPACE ${FRANKA_PANDA_LIB}::
    FILE ${CMAKE_BINARY_DIR}/${FRANKA_PANDA_TARGETS}.cmake)

# Register package in user registry
export(PACKAGE ${FRANKA_PANDA_LIB})

