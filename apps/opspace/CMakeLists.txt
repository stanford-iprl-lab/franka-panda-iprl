############################################################
# CMakeLists for the example Frank Panda Opspace Controller
#
# Copyright 2019. All Rights Reserved.
# Stanford IPRL
#
# Created: January 09, 2019
# Authors: Toki Migimatsu
############################################################

# Create the Franka Panda opspace controller.
set(BIN_NAME ${PROJECT_NAME}_opspace)

add_executable(${BIN_NAME} "src/main.cc")

target_compile_features(${BIN_NAME} PUBLIC cxx_std_14)
set_target_properties(${BIN_NAME} PROPERTIES CXX_EXTENSIONS OFF)

if(FRANKA_PANDA_CLANG_TIDY)
    target_enable_clang_tidy(${BIN_NAME})
endif()

lib_add_subdirectory(ctrl_utils)
ctrl_utils_add_subdirectory(cpp_redis)
lib_add_subdirectory(redis_gl)
lib_add_subdirectory(spatial_dyn INCLUDE_ALL_TARGETS)
target_link_libraries(${BIN_NAME}
  PRIVATE
    cpp_redis::cpp_redis
    ctrl_utils::ctrl_utils
    franka_panda::franka_panda
    redis_gl::redis_gl
    spatial_dyn::spatial_dyn
)
