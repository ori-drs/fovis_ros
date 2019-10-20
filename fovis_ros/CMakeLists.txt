cmake_minimum_required(VERSION 2.8.3)
project(fovis_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  fovis
  estimate_pose
  image_transport
  tf_conversions
  eigen_conversions
  cv_bridge
  fovis_msgs
  pcl_ros
)

find_package(OpenCV 3.0 QUIET)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fovis_ros
#  CATKIN_DEPENDS bot2_frames_cpp cv_bridge image_io_utils pronto_conversions pronto_vis
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################
set(LIBRARY_NAME voconfig)
add_library(${LIBRARY_NAME} src/voconfig/voconfig.cpp)
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})

################
set(LIBRARY_NAME vofeatures)
add_library(${LIBRARY_NAME} src/vofeatures/vofeatures.cpp)
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})

################
set(LIBRARY_NAME voestimator)
add_library(${LIBRARY_NAME} src/voestimator/voestimator.cpp)
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})

################
set(LIBRARY_NAME fovision)
add_library(${LIBRARY_NAME} src/fovision/fovision.cpp src/fovision/visualization.cpp)
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES})



set(LIBRARY_NAME fovision_fusion_core)
add_library(${LIBRARY_NAME} src/fovision_apps/fovision_fusion_core.cpp)
target_link_libraries(${LIBRARY_NAME} ${catkin_LIBRARIES} voconfig vofeatures voestimator fovision)


####### LCM Fusion #########
#add_executable(simple_fusion_lcm src/fovision_apps/fovision_fusion.cpp)
#target_link_libraries (simple_fusion_lcm ${catkin_LIBRARIES} fovision_fusion_core)

####### ROS Fusion #########
add_executable(simple_fusion src/fovision_apps/fovision_fusion_ros.cpp)
target_link_libraries (simple_fusion ${catkin_LIBRARIES} fovision_fusion_core)

####### LCM Basic VO #######
#add_executable(simple_vo_lcm src/fovision_apps/fovision_vo.cpp)
#target_link_libraries (simple_vo_lcm ${catkin_LIBRARIES} voconfig vofeatures voestimator fovision)



################
#set(LIBRARY_NAME vis-loop-closure)
#add_library(${LIBRARY_NAME} src/vis_loop_closure/vis_loop_closure.cpp)
#target_link_libraries (${LIBRARY_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS}
#  voconfig vofeatures voestimator fovision)

################
#add_executable(se-vis-loop-closure-batch src/vis_loop_closure/vis_loop_closure_batch.cpp)
#target_link_libraries (se-vis-loop-closure-batch ${catkin_LIBRARIES} ${OpenCV_LIBS}
#  vis-loop-closure)

################
#add_executable(se-vins-writer src/vins_writer/vins_writer.cpp)
#target_link_libraries (se-vins-writer ${catkin_LIBRARIES} ${OpenCV_LIBS} lcm)
