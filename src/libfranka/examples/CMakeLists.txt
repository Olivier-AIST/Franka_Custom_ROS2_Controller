cmake_minimum_required(VERSION 3.4)
project(libfranka-examples CXX)
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Load dependencies
add_subdirectory(/home/pbd/franka_ros2_ws/src/libfranka/include/ruckig ${CMAKE_BINARY_DIR}/ruckig_build)

include(FetchFMT)
if(NOT FRANKA_IS_FOUND)
 find_package(Franka REQUIRED)
endif()
find_package(Eigen3 REQUIRED)
# find_package(ruckig REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)
find_package(pinocchio REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
# Ruckig include and lib paths

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
 arm_sim2real
)
# target_sources(ruckig PRIVATE )
#   target_include_directories(ruckig PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
#     $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
#   )

foreach(example ${EXAMPLES})
 add_executable(${example} ${example}.cpp)
 target_compile_definitions(${example} PRIVATE WITH_CLOUD_CLIENT)

 target_include_directories(${example} PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 )
 

 target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3 pinocchio::pinocchio)
 if(${example} STREQUAL "arm_sim2real")
  target_link_libraries(${example} ruckig::ruckig)
endif()

endforeach()
target_compile_features(arm_sim2real PUBLIC cxx_std_17)




# Special linking for specific examples
target_link_libraries(joint_impedance_control Threads::Threads pinocchio::pinocchio)
target_link_libraries(motion_with_control Poco::Foundation)
target_link_libraries(motion_with_control_external_control_loop Poco::Foundation)

# ROS2 dependencies for arm_sim2real (after the target is created)
if(rclcpp_FOUND AND sensor_msgs_FOUND)
 target_link_libraries(arm_sim2real rclcpp::rclcpp sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
 target_compile_definitions(arm_sim2real PRIVATE -DROS2_AVAILABLE)
 target_link_libraries(grasp_object rclcpp::rclcpp sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
 target_compile_definitions(grasp_object PRIVATE -DROS2_AVAILABLE)
 message(STATUS "ROS2 dependencies found, building with ROS2 support")
else()
 message(WARNING "ROS2 dependencies not found, building without ROS2 support")
endif()

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
 LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
 RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Further examples
add_subdirectory(utility_examples)