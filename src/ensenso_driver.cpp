// ROS headers
#include <ros/ros.h>
//tf to rgb
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
// Conversions
#include <eigen_conversions/eigen_msg.h>
// Ensenso grabber
#include <ensenso/ensenso_grabber.h>
// Image transport
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <ensenso/CameraParametersConfig.h>
// Messages
#include <ensenso/RawStereoPattern.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
// Services
#include <ensenso/CalibrateHandEye.h>
#include <ensenso/CollectPattern.h>
#include <ensenso/EstimatePatternPose.h>
// boost
#include <boost/thread/thread.hpp>

// Typedefs
typedef std::pair<pcl::PCLGenImage<pcl::uint8_t>, pcl::PCLGenImage<pcl::uint8_t> > PairOfImages;
typedef pcl::PointXYZRGBA PointXYZRGBA;
typedef pcl::PointCloud<PointXYZRGBA> PointCloudXYZRGBA;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;


class EnsensoDriver
{
  private:
    // ROS
    ros::NodeHandle                   nh_, nh_private_;
    ros::ServiceServer                pattern_srv_;
    ros::ServiceServer                collect_srv_;
    ros::ServiceServer                calibrate_srv_;
    dynamic_reconfigure::Server<ensenso::CameraParametersConfig> reconfigure_server_;
    // Images
    image_transport::ImageTransport   it_;
    image_transport::CameraPublisher  rgb_raw_pub_;
    image_transport::CameraPublisher  l_raw_pub_;
    image_transport::CameraPublisher  r_raw_pub_;
    image_transport::CameraPublisher  depth_pub_;
    image_transport::Publisher        rgb_rectified_pub_;
    image_transport::Publisher        l_rectified_pub_;
    image_transport::Publisher        r_rectified_pub_;

    // Publishers
    ros::Publisher                    cloud_pub_;
    ros::Publisher                    pattern_pose_pub_;
    ros::Publisher                    pattern_raw_pub_;
    // Streaming configuration
    bool                              rgb_available_;
    bool                              enable_cloud_;
    bool                              enable_images_;
    bool                              enable_depth_;
    bool                              is_streaming_cloud_;
    bool                              is_streaming_depth_;
    bool                              is_streaming_images_;
    bool                              find_pattern_;
    bool                              stream_calib_pattern_;
    int                               trigger_mode_;
    // TF
    std::string                       camera_frame_id_;
    std::string                       rgb_camera_frame_id_;
    tf2_ros::TransformBr