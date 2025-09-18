cmake_minimum_required(VERSION 3.4)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Load dependencies
include(FetchFMT)

if(NOT FRANKA_IS_FOUND)
    find_package(Franka REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)
find_package(pinocchio REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)
target_include_directories(examples_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set(EXAMPLES
  cartesian_impedance_control
  communication_test
  echo_robot_state
  force_control
  generate_cartesian_pose_motion
  generate_cartesian_pose_motion_external_control_loop
  generate_cartesian_velocity_motion
  generate_cartesian_velocity_motion_external_control_loop
  generate_consecutive_motions
  generate_elbow_motion
  generate_joint_position_motion
  generate_joint_position_motion_external_control_loop
  generate_joint_velocity_motion
  generate_joint_velocity_motion_external_control_loop
  grasp_object
  joint_impedance_control
  joint_point_to_point_motion
  motion_with_control
  motion_with_control_external_control_loop
  print_joint_poses
  vacuum_object
)

foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_include_directories(${example} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3 pinocchio::pinocchio nlohmann_json::nlohmann_json)
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads pinocchio::pinocchio)
target_link_libraries(motion_with_control Poco::Foundation)
target_link_libraries(motion_with_control_external_control_loop Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Further examples
add_subdirectory(utility_examples)


set(CMAKE_PREFIX_PATH "/home/hex/Documents/github/rfl/FrankaAssembly/.venv/lib/python3.10/site-packages/nanobind/cmake")
if (CMAKE_VERSION VERSION_LESS 3.18)
    set(DEV_MODULE Development)
else()
    set(DEV_MODULE Development.Module)
endif()

# Set Python executable explicitly
set(Python_EXECUTABLE "/home/hex/Documents/github/rfl/FrankaAssembly/.venv/bin/python")
find_package(Python 3.10 COMPONENTS Interpreter ${DEV_MODULE} REQUIRED)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)

cmake_policy(SET CMP0074 NEW)
find_package(nanobind CONFIG REQUIRED)
nanobind_add_module(frankz frankz.cpp)
target_link_libraries(frankz PRIVATE Franka::Franka examples_common Eigen3::Eigen3 pinocchio::pinocchio nlohmann_json::nlohmann_json)
