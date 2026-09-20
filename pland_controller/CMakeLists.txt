cmake_minimum_required(VERSION 3.14)
project(pland_controller LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  mavros_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GeographicLib REQUIRED geographiclib)

# ------------------------------------------------------------------------------
# 第三方依赖
# ------------------------------------------------------------------------------
set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../.deps CACHE PATH "Directory for fetched content")
include(FetchContent)

# Eigen3 (Header-only)
FetchContent_Declare(eigen GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git GIT_TAG 3.4.0 GIT_SHALLOW TRUE)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
  FetchContent_Populate(eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()

# ros_param_sync (动态参数持久化组件，支持 FetchContent)
FetchContent_Declare(ros_param_sync GIT_REPOSITORY https://github.com/HGGshiwo/ros_param_sync.git GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(ros_param_sync)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pland_controller_lib
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs std_msgs sensor_msgs mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_library(pland_controller_lib
  src/pland_controller.cpp
)
target_link_libraries(pland_controller_lib
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ros_param_sync::ros_param_sync
  Eigen3::Eigen
)

add_executable(pland_controller_node
  src/pland_controller_node.cpp
)
target_link_libraries(pland_controller_node
  pland_controller_lib
  ros_param_sync::ros_param_sync
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  Eigen3::Eigen
)
