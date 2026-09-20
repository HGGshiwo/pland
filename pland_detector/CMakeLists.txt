cmake_minimum_required(VERSION 3.14)
project(pland_detector LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------------
# 1. ROS 依赖
# ------------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------
# 2. 第三方依赖 (复用工作空间已有的 .deps 目录)
# ------------------------------------------------------------------------------
set(FETCHCONTENT_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../.deps CACHE PATH "Directory for fetched content")
include(FetchContent)

# apriltag
FetchContent_Declare(apriltag GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git GIT_TAG v3.3.0 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(apriltag)

# Eigen3 (Header-only)
FetchContent_Declare(eigen GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git GIT_TAG 3.4.0 GIT_SHALLOW TRUE)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
  FetchContent_Populate(eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  target_include_directories(Eigen3::Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()

# nlohmann_json (Header-only)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
FetchContent_Declare(nlohmann_json GIT_REPOSITORY https://github.com/nlohmann/json.git GIT_TAG v3.11.2 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(nlohmann_json)

# ros_param_sync (动态参数持久化组件)
FetchContent_Declare(ros_param_sync GIT_REPOSITORY https://github.com/HGGshiwo/ros_param_sync.git GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(ros_param_sync)

# ------------------------------------------------------------------------------
# 3. Catkin 配置
# ------------------------------------------------------------------------------
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pland_detector_lib
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs std_msgs std_srvs cv_bridge image_transport
)

include_directories(
  include
  include/tag_detector
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------
# 4. 构建目标
# ------------------------------------------------------------------------------
# Pland detector 核心动态库
add_library(pland_detector_lib
  src/pland_detector.cpp
)
target_link_libraries(pland_detector_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  apriltag
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
  ros_param_sync::ros_param_sync
)

# Pland detector ROS 节点
add_executable(pland_detector_node
  src/pland_detector_node.cpp
)
target_link_libraries(pland_detector_node
  pland_detector_lib
  ros_param_sync::ros_param_sync
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  apriltag
  Eigen3::Eigen
)
