cmake_minimum_required(VERSION 2.8.3)
project(nautilus_worlds)

#enable C++11 support
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    rospy
    gazebo_ros
    gazebo_plugins
    gazebo_msgs
    std_msgs
)

catkin_package(
  DEPENDS gazebo_ros gazebo_plugins roscpp
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})
file (GLOB_RECURSE FILES *.cpp *.h *.hpp)

### Build thruster plugin
add_library(thruster SHARED src/thruster_manager.cc)
target_link_libraries( thruster ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} )

### Make Python Stuff
catkin_install_python(PROGRAMS
  scripts/sample_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
