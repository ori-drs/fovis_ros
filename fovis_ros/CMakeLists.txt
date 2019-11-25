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
  pcl_ros
  eigen_utils
)

find_package(OpenCV 3.0 QUIET)

set(VO_CONFIG_LIBNAME voconfig)
set(VO_FEATURES_LIBNAME vofeatures)
set(VO_ESTIMATOR_LIBNAME voestimator)
set(FOVISION_LIBNAME fovision)
set(FOVISION_FUSION_LIBNAME fovision_fusion_core)
set(FOVIS_EXE simple_fusion)

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

add_executable(${FOVIS_EXE} src/fovision_apps/fovision_fusion_ros.cpp
                             src/fovision_apps/stereo_odom_ros.cpp)
target_link_libraries (${FOVIS_EXE} ${catkin_LIBRARIES} ${FOVISION_FUSION_LIBNAME})

install(TARGETS ${VO_CONFIG_LIBNAME}
                ${VO_FEATURES_LIBNAME}
                ${VO_ESTIMATOR_LIBNAME}
                ${FOVISION_LIBNAME}
                ${FOVISION_FUSION_LIBNAME}
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS ${FOVIS_EXE}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS python/fake_imu_horizontal.py  
	                       python/fake_imu.py
			       python/static_transform_publisher_using_message_time.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
