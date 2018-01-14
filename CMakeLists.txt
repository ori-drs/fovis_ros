cmake_minimum_required(VERSION 2.8.3)
project(vo_estimator)

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pronto_vis
  image_io_utils
  pronto_conversions
  bot_frames_cpp
  pronto_lcmtypes
  reg_lcmtypes
  bot_core_lcmtypes
  ori_lcmtypes
  bot_param
  fovis
  concise_args
  path_util
  fovis_lcmtypes
  estimate_pose
)

find_package(OpenCV 3.0 QUIET)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vo_estimator
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



################
add_executable(se-simple-fusion src/fovision_apps/fovision_fusion.cpp)
target_link_libraries (se-simple-fusion ${catkin_LIBRARIES}
  voconfig vofeatures voestimator fovision)

################
add_executable(se-simple-vo src/fovision_apps/fovision_vo.cpp)
target_link_libraries (se-simple-vo ${catkin_LIBRARIES}
  voconfig vofeatures voestimator fovision)



################
set(LIBRARY_NAME vis-loop-closure)
add_library(${LIBRARY_NAME} src/vis_loop_closure/vis_loop_closure.cpp)
target_link_libraries (${LIBRARY_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS}
  voconfig vofeatures voestimator fovision)

################
add_executable(se-vis-loop-closure-batch src/vis_loop_closure/vis_loop_closure_batch.cpp)
target_link_libraries (se-vis-loop-closure-batch ${catkin_LIBRARIES} ${OpenCV_LIBS}
  vis-loop-closure)

################
add_executable(se-vins-writer src/vins_writer/vins_writer.cpp)
target_link_libraries (se-vins-writer ${catkin_LIBRARIES} ${OpenCV_LIBS} lcm)