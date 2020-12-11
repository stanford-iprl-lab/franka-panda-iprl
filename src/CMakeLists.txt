############################################################
# CMakeLists for the Franka Panda driver.
#
# Copyright 2020. All Rights Reserved.
# Stanford IPRL
#
# Created: December 9, 2020
# Authors: Toki Migimatsu
############################################################

set(LIB_NAME ${PROJECT_NAME})
set(BIN_NAME ${PROJECT_NAME}_driver)
set(LIB_EXTERNAL_DIR "${PROJECT_SOURCE_DIR}/external")

# Create the Franka Panda model library.
add_library(${LIB_NAME} SHARED)
add_library(${LIB_NAME}::${LIB_NAME} ALIAS ${LIB_NAME})

target_compile_features(${LIB_NAME} PUBLIC cxx_std_14)
set_target_properties(${LIB_NAME} PROPERTIES CXX_EXTENSIONS OFF)

if(FRANKA_PANDA_CLANG_TIDY)
    target_enable_clang_tidy(${LIB_NAME})
endif()

set(LIB_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
target_include_directories(${LIB_NAME}
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${LIB_INCLUDE_DIR}>
  PRIVATE
    ${LIB_EXTERNAL_DIR}/Franka/libfranka.git/src
)

target_sources(${LIB_NAME}
  PRIVATE
    model.cc
)

# Link internal model library.
find_library(FCI_MODELS_LIB fcimodels REQUIRED PATHS ${LIB_EXTERNAL_DIR})
lib_add_subdirectory(ctrl_utils INCLUDE_ALL_TARGETS)
ctrl_utils_add_subdirectory(Eigen3)
ctrl_utils_add_subdirectory(nlohmann_json)
target_link_libraries(${LIB_NAME}
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    ctrl_utils::ctrl_utils
    ${FCI_MODELS_LIB}
    nlohmann_json::nlohmann_json
)

# Create the Franka Panda driver.
add_executable(${BIN_NAME} "")

target_compile_features(${BIN_NAME} PUBLIC cxx_std_14)
set_target_properties(${BIN_NAME} PROPERTIES CXX_EXTENSIONS OFF)

if(FRANKA_PANDA_CLANG_TIDY)
    target_enable_clang_tidy(${BIN_NAME})
endif()

target_include_directories(${BIN_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(${BIN_NAME}
  PRIVATE
    main.cc
    args.cc
    cartesian_pose_control.cc
    control_thread.cc
    gripper_thread.cc
    redis_thread.cc
    torque_control.cc
)

ctrl_utils_add_subdirectory(cpp_redis)
ctrl_utils_add_subdirectory(yaml-cpp)
lib_add_subdirectory(Franka)
target_link_libraries(${BIN_NAME}
  PRIVATE
    atomic  # Needed for gcc 5.4.0
    cpp_redis::cpp_redis
    ctrl_utils::ctrl_utils
    Eigen3::Eigen
    Franka::Franka
    nlohmann_json::nlohmann_json
    yaml-cpp::yaml-cpp
)
