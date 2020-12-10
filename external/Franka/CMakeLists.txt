############################################################
# CMakeLists for libfranka.
#
# Copyright 2020. All Rights Reserved.
#
# Created: December 9, 2020
# Authors: Toki Migimatsu
############################################################

if(NOT TARGET Franka::Franka)
    option(BUILD_TESTS "Build libfranka tests" OFF)
    option(BUILD_EXAMPLES "Build libfranka examples" OFF)

    init_git_submodule(libfranka.git RECURSIVE)

    get_target_property(EIGEN3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
    execute_process(
        COMMAND sed -i "s/CMAKE_SOURCE_DIR/PROJECT_SOURCE_DIR/g" libfranka.git/CMakeLists.txt
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    )
    add_subdirectory(libfranka.git)
endif()
