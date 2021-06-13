cmake_minimum_required(VERSION 3.0.2)
project(fovis_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS message_generation
                                        std_msgs
                                        geometry_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    Stats.msg
    VisualOdometryUpdate.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs
                               geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
