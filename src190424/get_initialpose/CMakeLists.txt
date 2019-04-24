cmake_minimum_required(VERSION 2.8.3)
project(get_initialpose)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES get_initialpose
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(amcl_initialpose src/amcl_initialpose.cpp)
target_link_libraries(amcl_initialpose ${catkin_LIBRARIES})

add_executable(record_pose src/record_pose.cpp)
target_link_libraries(record_pose ${catkin_LIBRARIES})


