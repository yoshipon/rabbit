cmake_minimum_required(VERSION 2.8.3)
project(rabbit_rqt_plugins)

find_package(catkin REQUIRED std_msgs rabbit_msgs)

catkin_package(
  CATKIN_DEPENDS std_msgs rabbit_msgs
)