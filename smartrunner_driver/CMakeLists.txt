# SPDX-License-Identifier: Apache-2.0
#
# Copyright 2025 Pepperl+Fuchs SE
#
# Authors:
#   Adrian Krzizok <git@breeze-innovations.com>
#   Jan Hegner <opensource-jhe@de.pepperl-fuchs.com>
#   Markus Moll <opensource-mmo@de.pepperl-fuchs.com>

cmake_minimum_required(VERSION 3.14.4)
project(pepperl_fuchs_smartrunner)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(VsxSdk REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(smartrunner_node_base OBJECT
  src/rosnode/SmartRunnerNodeBase.cpp
  src/rosnode/Utilities.cpp
)
  
ament_target_dependencies(smartrunner_node_base
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(smartrunner_node_base VsxSdk)

add_executable(smartrunner_2d_node
  src/rosnode/SmartRunner2dNode.cpp
)

add_executable(smartrunner_3d_stereo_node
  src/rosnode/SmartRunner3dStereoNode.cpp
)

add_executable(smartrunner_3d_tof_node
  src/rosnode/SmartRunner3dTofNode.cpp
)

set(targets
  smartrunner_2d_node
  smartrunner_3d_stereo_node
  smartrunner_3d_tof_node
)

ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
)

foreach(tgt IN LISTS targets)
  target_include_directories(${tgt} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  )

  target_link_libraries(${tgt} smartrunner_node_base)

  install(TARGETS ${tgt}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(FILES rviz/PointCloud2.rviz
  DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()
