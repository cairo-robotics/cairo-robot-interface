cmake_minimum_required(VERSION 2.8.3)
project(cairo_sawyer_interface)

## Compile as C++11, supported in ROS Kinetic and newer
##c++11 required for moveit
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  moveit_msgs
  message_generation
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  sensor_msgs
  shape_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  forward_kinematics.srv
)
###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(forward_kinematic_node src/forward_kinematic_node.cpp)
target_link_libraries(forward_kinematic_node
                      ${catkin_LIBRARIES})

add_executable(forward_kinematic_server src/forward_kinematic_server.cpp)
target_link_libraries(forward_kinematic_server
                      ${catkin_LIBRARIES})

install(TARGETS forward_kinematic_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS forward_kinematic_server
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
