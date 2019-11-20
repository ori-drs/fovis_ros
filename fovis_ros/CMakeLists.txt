cmake_minimum_required(VERSION 2.8.3)
project(fovis_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  fovis
  #  estimate_pose
  image_transport
  tf_conversions
  eigen_conversions
  fovis_msgs
)

find_package(OpenCV 3.0 QUIET)

set(VO_CONFIG_LIBNAME voconfig)
set(VO_FEATURES_LIBNAME vofeatures)
set(VO_ESTIMATOR_LIBNAME voestimator)
set(FOVISION_LIBNAME fovision)
set(FOVISION_FUSION_LIBNAME fovision_fusion_core)

catkin_package(INCLUDE_DIRS include
	       LIBRARIES ${VO_CONFIG_LIBNAME} 
	                 ${VO_FEATURES_LIBNAME}
			 ${VO_ESTIMATOR_LIBNAME}
			 ${FOVISION_LIBNAME}
			 ${FOVISION_FUSION_LIBNAME}
               CATKIN_DEPENDS cv_bridge 
	                      fovis 
			      # estimate_pose
			      image_transport
			      tf_conversions
			      eigen_conversions
			      fovis_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${VO_CONFIG_LIBNAME} src/voconfig/voconfig.cpp)
target_link_libraries(${VO_CONFIG_LIBNAME} ${catkin_LIBRARIES})

add_library(${VO_FEATURES_LIBNAME} src/vofeatures/vofeatures.cpp)
target_link_libraries(${VO_FEATURES_LIBNAME} ${catkin_LIBRARIES})

add_library(${VO_ESTIMATOR_LIBNAME} src/voestimator/voestimator.cpp)
target_link_libraries(${VO_ESTIMATOR_LIBNAME} ${catkin_LIBRARIES})

add_library(${FOVISION_LIBNAME} src/fovision/fovision.cpp
                                src/fovision/visualization.cpp)
target_link_libraries(${FOVISION_LIBNAME} ${catkin_LIBRARIES})

add_library(${FOVISION_FUSION_LIBNAME} src/fovision_apps/fovision_fusion_core.cpp)
target_link_libraries(${FOVISION_FUSION_LIBNAME} ${catkin_LIBRARIES} ${VO_CONFIG_LIBNAME} ${VO_FEATURES_LIBNAME} ${VO_ESTIMATOR_LIBNAME} ${FOVISION_LIBNAME})

add_executable(simple_fusion src/fovision_apps/fovision_fusion_ros.cpp
                             src/fovision_apps/stereo_odom_ros.cpp)
target_link_libraries (simple_fusion ${catkin_LIBRARIES} ${FOVISION_FUSION_LIBNAME})

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
