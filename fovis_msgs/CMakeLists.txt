cmake_minimum_required(VERSION 2.8.3)
project(fovis_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
#  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Stats.msg
)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  ExecuteTaskArray.srv
#  Request.srv
#  RequestTaskArray.srv
#)

## Generate actions in the 'action' folder
#add_action_files(
#   FILES
#   ExecuteRobotTaskArray.action
#   PoseNamedApprove.action
#   ContinueApprove.action
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
#  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs #geometry_msgs free_gait_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
