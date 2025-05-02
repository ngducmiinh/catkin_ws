cmake_minimum_required(VERSION 3.0.2)
project(custom_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs nav_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add any nodes you want to build here
# For example:
# add_executable(your_node src/your_node.cpp)
# target_link_libraries(your_node ${catkin_LIBRARIES})

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)