cmake_minimum_required(VERSION 2.8.3)
project(onboard_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_msgs
  uav_msgs
)
catkin_package(
    CATKIN_DEPENDS std_msgs roscpp
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${onboard_controller_SOURCE_DIR} include/onboard_controller
)

add_executable(
  onboard_controller_node
  src/onboard_controller_node.cpp
  src/onboard_controller.cpp
)
target_link_libraries(onboard_controller_node ${catkin_LIBRARIES})

add_dependencies(onboard_controller_node ${catkin_EXPORTED_TARGETS} )

