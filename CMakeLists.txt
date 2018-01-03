cmake_minimum_required(VERSION 2.8.3)
project(ned_tools)

find_package(catkin REQUIRED COMPONENTS
  auv_msgs
  cmake_modules
  geometry_msgs
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_python_setup()

# Dependencies - Eigen:
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy auv_msgs sensor_msgs geometry_msgs std_msgs std_srvs cmake_modules message_filters
  INCLUDE_DIRS include
  LIBRARIES ned_tools)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_library(ned_tools src/ned.cpp src/nav_utils.cpp)
add_dependencies(ned_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ned_tools ${catkin_LIBRARIES})

install(PROGRAMS
  src/ned_tools/NED.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)