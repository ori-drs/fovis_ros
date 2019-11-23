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
#include <pronto_msgs/VisualOdometryUpdate.h>

struct StereoOdomConfig {
  std::string output_tf_frame;
  bool write_pose_to_file;
};

/**
 * @brief ROS wrapper for the FusionCore class. Takes stereo images, IMU, and
 * pose messages and forwards them to FusionCore. Then takes the outputs and
 * publishes useful messages such as statistics, TF and pose messages
 */
class StereoOdom{
  public:
    StereoOdom(ros::NodeHandle& node_in,
               const StereoOdomConfig& cfg,
               const FusionCoreConfig& fcfg);

    virtual ~StereoOdom() = default;
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

    void publishDeltaVO();

  private:
    ros::NodeHandle& node_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter image_a_ros_sub_, image_b_ros_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_a_ros_sub_, info_b_ros_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                      sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync_without_info_;

    StereoOdomConfig cfg_;
    const FusionCoreConfig fcfg_;
    std::unique_ptr<FusionCore> vo_core_;

    tf::TransformBroadcaster br;

    ros::Subscriber imuSensorSub_;
    ros::Subscriber poseOdomSub_;

    ros::Publisher body_pose_pub_;
    ros::Publisher fovis_stats_pub_;
    ros::Publisher features_image_pub_;
    ros::Publisher features_cloud_pub_;
    ros::Publisher delta_vo_pub_;

    int64_t utime_imu_;

    bool output_using_imu_time_;
    int stereo_counter = 0;

    pronto_msgs::VisualOdometryUpdate delta_vo_msg_;
private:
    int convertPixelRGBtoGray(uint8_t *dest,
                              int dstride,
                              int width,
                              int height,
                              const uint8_t *src,
                              int sstride);
};
