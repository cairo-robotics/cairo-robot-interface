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
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  sensor_msgs
  shape_msgs
)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ForwardKinematics.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    message_runtime
 )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(forward_kinematics ${PROJECT_SOURCE_DIR}/src/forward_kinematics/forward_kinematics.cpp)
target_link_libraries(forward_kinematics ${catkin_LIBRARIES})


add_executable(forward_kinematics_node ${PROJECT_SOURCE_DIR}/src/forward_kinematics/forward_kinematics_node.cpp)
add_dependencies(forward_kinematics_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(forward_kinematics_node ${catkin_LIBRARIES})

add_executable(forward_kinematics_service ${PROJECT_SOURCE_DIR}/src/forward_kinematics/forward_kinematics_service.cpp)
add_dependencies(forward_kinematics_service ${catkin_EXPORTED_TARGETS})
target_link_libraries(forward_kinematics_service forward_kinematics)

add_executable(path_constraint ${PROJECT_SOURCE_DIR}/src/path_constraints/path_constraint_test.cpp)
add_dependencies(path_constraint ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_constraint ${catkin_LIBRARIES})



install(TARGETS forward_kinematics_node forward_kinematics_node path_constraint
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
