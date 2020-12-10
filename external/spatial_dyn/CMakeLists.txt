############################################################
# CMakeLists for spatial_dyn.
#
# Copyright 2020. All Rights Reserved.
#
# Created: September 23, 2020
# Authors: Toki Migimatsu
############################################################

if(NOT TARGET spatial_dyn::spatial_dyn)
    init_git_submodule(spatial-dyn.git)
    add_subdirectory(spatial-dyn.git)
endif()
