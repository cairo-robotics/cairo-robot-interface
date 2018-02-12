cmake_minimum_required(VERSION 2.8.3)
project(cairo_sawyer_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  sensor_msgs
  shape_msgs
  tf
)

catkin_python_setup()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## forward kinematics server build ##
add_executable(forward_kinematic_server src/forward_kinematic_server.cpp)
target_link_libraries(forward_kinematic_server
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})

#install(TARGETS forward_kinematic_server RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
