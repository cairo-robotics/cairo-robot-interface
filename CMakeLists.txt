cmake_minimum_required(VERSION 3.0.2)
project(cairo_robot_interface)

## Compile as C++11, supported in ROS Kinetic and newer
##c++11 required for moveit
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  moveit_msgs
  tf2_msgs
)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  TransformLookup.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  moveit_msgs
  tf2_msgs
)

catkin_package(
  CATKIN_DEPENDS
 )

###########
## Build ##
###########

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${Boost_INCLUDE_DIRS}
# )

# add_library(forward_kinematics ${PROJECT_SOURCE_DIR}/src/forward_kinematics/forward_kinematics.cpp)
# target_link_libraries(forward_kinematics ${catkin_LIBRARIES})

# add_executable(forward_kinematics_node ${PROJECT_SOURCE_DIR}/src/forward_kinematics/forward_kinematics_node.cpp)
# add_dependencies(forward_kinematics_node ${catkin_EXPORTED_TARGETS})
# target_link_libraries(forward_kinematics_node ${catkin_LIBRARIES})

# add_executable(forward_kinematics_service ${PROJECT_SOURCE_DIR}/src/forward_kinematics/forward_kinematics_service.cpp)
# add_dependencies(forward_kinematics_service ${catkin_EXPORTED_TARGETS})
# target_link_libraries(forward_kinematics_service forward_kinematics)

# install(TARGETS forward_kinematics_node
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install(TARGETS forward_kinematics_service
#         RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
