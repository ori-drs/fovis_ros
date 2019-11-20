#pragma once
#include "fovision_apps/fovision_fusion_core.hpp"
#include <ros/node_handle.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class StereoOdom{
  public:
    StereoOdom(ros::NodeHandle node_in,
               FusionCoreConfig fcfg);

    virtual ~StereoOdom();
private:
    void stereoWithInfoCallback(const sensor_msgs::ImageConstPtr& image_a_ros,
                        const sensor_msgs::CameraInfoConstPtr& info_cam_a,
                        const sensor_msgs::ImageConstPtr& image_b_ros,
                        const sensor_msgs::CameraInfoConstPtr& info_cam_b);

    void stereoCallback(const sensor_msgs::ImageConstPtr& image_a_ros,
                                     const sensor_msgs::ImageConstPtr& image_b_ros);

    void imuSensorCallback(const sensor_msgs::ImuConstPtr& msg);

    void poseOdomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    void publishFovisStats(int sec, int nsec);

  private:
    ros::NodeHandle node_;
    image_transport::ImageTransport it_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                      sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync_without_info_;
    const FusionCoreConfig fcfg_;
    FusionCore* vo_core_;

    image_transport::SubscriberFilter image_a_ros_sub_, image_b_ros_sub_;

    message_filters::Subscriber<sensor_msgs::CameraInfo> info_a_ros_sub_, info_b_ros_sub_;

    tf::TransformBroadcaster br;

    ros::Subscriber imuSensorSub_;
    ros::Subscriber poseOdomSub_;

    ros::Publisher body_pose_pub_, fovis_stats_pub_;
    ros::Publisher features_image_pub_;
    ros::Publisher features_cloud_pub_;

    int64_t utime_imu_;

    bool output_using_imu_time_;
    int stereo_counter = 0;
};
