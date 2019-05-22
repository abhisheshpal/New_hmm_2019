cmake_minimum_required(VERSION 2.8.3)
project(rasberry_people_perception)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #message_generation
  geometry_msgs
  people_msgs
  roscpp
  rospy
  std_msgs
  strands_navigation_msgs
  tf
  topological_navigation
)

 catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   PeopleStamped.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rasberry_people_perception
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS
  scripts/simple_marvel_localiser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ${USE_SOURCE_PERMISSIONS}
)


install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


