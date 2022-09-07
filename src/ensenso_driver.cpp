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
    tf2_ros::TransformBroadcaster     tf_br_;
    ros::Timer                        tf_publisher_;
    // Ensenso grabber
    boost::signals2::connection       cloud_connection_;
    boost::signals2::connection       image_connection_;
    boost::signals2::connection       depth_connection_;
    pcl::EnsensoGrabber::Ptr          ensenso_ptr_;

  public:
     EnsensoDriver():
      rgb_available_(false),
      enable_cloud_(false),
      enable_images_(false),
      enable_depth_(false),
      is_streaming_images_(false),
      is_streaming_cloud_(false),
      is_streaming_depth_(false),
      stream_calib_pattern_(false),
      find_pattern_(true),
      trigger_mode_(-1),
      nh_private_("~"),
      it_(nh_)
    {
      // Read parameters
      std::string serial, monoserial;
      nh_private_.param(std::string("serial"), serial, std::string("150534"));
      if (!nh_private_.hasParam("serial"))
        ROS_WARN_STREAM("Parameter [~serial] not found, using default: " << serial);
      nh_private_.param(std::string("monoserial"), monoserial, std::string("4103203953"));
      if (!nh_private_.hasParam("monoserial"))
        ROS_WARN_STREAM("Parameter [~monoserial] not found, using default: " << monoserial);
      nh_private_.param("camera_frame_id", camera_frame_id_, std::string("ensenso_optical_frame"));
      if (!nh_private_.hasParam("camera_frame_id"))
        ROS_WARN_STREAM("Parameter [~camera_frame_id] not found, using default: " << camera_frame_id_);
      nh_private_.param("rgb_camera_frame_id", rgb_camera_frame_id_, std::string("ensenso_rgb_optical_frame"));
      if (!nh_private_.hasParam("rgb_camera_frame_id"))
        ROS_WARN_STREAM("Parameter [~rgb_camera_frame_id] not found, using default: " << rgb_camera_frame_id_);
      nh_private_.param("stream_calib_pattern", stream_calib_pattern_, false);
      if (!nh_private_.hasParam("stream_calib_pattern"))
        ROS_WARN_STREAM("Parameter [~stream_calib_pattern] not found, using default: " << (stream_calib_pattern_ ? "TRUE":"FALSE"