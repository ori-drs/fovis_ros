cmake_minimum_required(VERSION 2.8.3)
# needed for string arrays
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

if (NOT COMMAND pods_use_pkg_config_packages)
  include(cmake/pods.cmake)
endif()

project(vo_estimator)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pronto_vis
  image_io_utils
  pronto_conversions
  bot2_frames_cpp
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS cv_bridge pronto_vis image_io_utils
    pronto_conversions bot2_frames_cpp
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

################
set(LIBRARY_NAME voconfig)
add_library(${LIBRARY_NAME} src/voconfig/voconfig.cpp)
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})
pods_use_pkg_config_packages(${LIBRARY_NAME} bot2-param-client libfovis)

################
set(LIBRARY_NAME vofeatures)
add_library(${LIBRARY_NAME} src/vofeatures/vofeatures.cpp)
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})
#pods_use_pkg_config_packages(${LIBRARY_NAME} bot2-param-client libfovis)

################
set(LIBRARY_NAME voestimator)
add_library(${LIBRARY_NAME} src/voestimator/voestimator.cpp)
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})

################
set(LIBRARY_NAME fovision)
add_library(${LIBRARY_NAME} src/fovision/fovision.cpp src/fovision/visualization.cpp)
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})
pods_use_pkg_config_packages(${LIBRARY_NAME} lcmtypes_bot2-core)# libfovis)



################
add_executable(se-simple-fusion src/fovision_apps/fovision_fusion.cpp)
target_link_libraries (se-simple-fusion ${catkin_LIBRARIES}
  voconfig vofeatures voestimator fovision 
  image_io_utils pronto_math pronto_vis jpeg_utils z boost_system ${OpenCV_LIBRARIES} )
pods_use_pkg_config_packages(se-simple-fusion
  bot2-frames bot2-lcmgl-client path-util)

################
add_executable(se-simple-vo src/fovision_apps/fovision_vo.cpp)
target_link_libraries (se-simple-vo ${catkin_LIBRARIES}
  voconfig vofeatures voestimator fovision 
  image_io_utils pronto_math pronto_vis jpeg_utils z boost_system ${OpenCV_LIBRARIES} )
pods_use_pkg_config_packages(se-simple-vo
  bot2-frames bot2-lcmgl-client path-util)



################
set(LIBRARY_NAME vis-loop-closure)
add_library(${LIBRARY_NAME} src/vis_loop_closure/vis_loop_closure.cpp)
target_link_libraries (${LIBRARY_NAME} ${catkin_LIBRARIES} 
  vofeatures voconfig bot2_frames_cpp
  pronto_vis pronto_math image_io_utils jpeg_utils)
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})
pods_use_pkg_config_packages(${LIBRARY_NAME}
  estimate-pose)

################
add_executable(se-vis-loop-closure-batch src/vis_loop_closure/vis_loop_closure_batch.cpp)
target_link_libraries (se-vis-loop-closure-batch ${catkin_LIBRARIES}
  vis-loop-closure ${OpenCV_LIBRARIES} )
pods_use_pkg_config_packages(se-vis-loop-closure-batch
  bot2-frames bot2-lcmgl-client path-util)

################
add_executable(se-vins-writer src/vins_writer/vins_writer.cpp)
target_link_libraries (se-vins-writer ${catkin_LIBRARIES}
  image_io_utils ${OpenCV_LIBRARIES} jpeg_utils)

#############
## Install ##
#############

install(TARGETS ${TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
