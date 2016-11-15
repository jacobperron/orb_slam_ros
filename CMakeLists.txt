cmake_minimum_required(VERSION 2.4.6)
project(orb_slam_ros)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(orb_slam REQUIRED)
find_package(OpenCV 2.4.3 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  image_transport
  cv_bridge
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs image_transport cv_bridge tf2_geometry_msgs
  DEPENDS orb_slam OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
)

set(LIBRARIES
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  orb_slam::orb_slam
  ${Pangolin_LIBRARIES}
)

# Node for monocular camera
add_executable(mono_slam_node
  src/mono_slam_node.cpp
)

target_link_libraries(mono_slam_node
  ${LIBRARIES}
)

# TODO(jacobperron): Node for stereo camera
#add_executable(stereo_slam_node
#  src/stereo_slam_node.cpp
#)

#target_link_libraries(stereo_slam_node
#  ${LIBRARIES}
#)

# TODO(jacobperron): Node for RGB-D camera
#add_executable(rgbd_slam_node
#  src/rgbd_slam_node.cpp
#)

#target_link_libraries(rgbd_slam_node
#  ${LIBRARIES}
#)

