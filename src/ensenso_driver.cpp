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
        ROS_WARN_STREAM("Parameter [~stream_calib_pattern] not found, using default: " << (stream_calib_pattern_ ? "TRUE":"FALSE"));
      // Advertise topics
      ros::SubscriberStatusCallback cloud_rssc = boost::bind(&EnsensoDriver::cloudSubscribeCallback, this);
      image_transport::SubscriberStatusCallback image_issc = boost::bind(&EnsensoDriver::imagesSubscribeCallback, this);
      ros::SubscriberStatusCallback image_rssc = boost::bind(&EnsensoDriver::imagesSubscribeCallback, this);
      image_transport::SubscriberStatusCallback depth_issc = boost::bind(&EnsensoDriver::depthSubscribeCallback, this);
      ros::SubscriberStatusCallback depth_rssc = boost::bind(&EnsensoDriver::depthSubscribeCallback, this);

      l_raw_pub_ = it_.advertiseCamera("left/image_raw", 1, image_issc, image_issc, image_rssc, image_rssc);
      r_raw_pub_ = it_.advertiseCamera("right/image_raw", 1, image_issc, image_issc, image_rssc, image_rssc);

      l_rectified_pub_ = it_.advertise("left/image_rect", 1, image_issc, image_issc);
      r_rectified_pub_ = it_.advertise("right/image_rect", 1, image_issc, image_issc);

      depth_pub_ = it_.advertiseCamera("depth/image_rect", 1, depth_issc, depth_issc, depth_rssc, depth_rssc);

      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2 >("depth/points", 1, cloud_rssc, cloud_rssc);

      // Initialize Ensenso
      ensenso_ptr_.reset(new pcl::EnsensoGrabber);
      ensenso_ptr_->openDevice(serial);
      try
      {
        if (ensenso_ptr_->openMonoDevice(monoserial))
        {
          rgb_available_ = true;
          ROS_INFO("Found RGB camera");
          image_transport::SubscriberStatusCallback image_issc = boost::bind(&EnsensoDriver::imagesSubscribeCallback, this);
          ros::SubscriberStatusCallback image_rssc = boost::bind(&EnsensoDriver::imagesSubscribeCallback, this);
          rgb_raw_pub_ = it_.advertiseCamera("rgb/image_raw", 1, image_issc, image_issc, image_rssc, image_rssc);
          rgb_rectified_pub_ = it_.advertise("rgb/image_rect_color", 1, image_issc, image_issc);
          tf_publisher_ = nh_.createTimer(ros::Duration(0.5), boost::bind(&EnsensoDriver::publishTF, this));
          ensenso_ptr_->setUseRGB(true);
        }
      }
      catch (pcl::IOException e)
      {
        rgb_available_ = false;
        ROS_INFO("No RGB camera found");
      }

      ensenso_ptr_->openTcpPort();
      ensenso_ptr_->storeCalibrationPattern(stream_calib_pattern_);
      // Start dynamic reconfigure server
      dynamic_reconfigure::Server<ensenso::CameraParametersConfig>::CallbackType f;
      f = boost::bind(&EnsensoDriver::cameraParametersCallback, this, _1, _2);
      reconfigure_server_.setCallback(f);
      // Start the camera.
      ensenso_ptr_->start();
      // Advertise services
      ROS_INFO("Finished [ensenso_driver] initialization");
    }

    ~EnsensoDriver()
    {
      cloud_connection_.disconnect();
      image_connection_.disconnect();
      depth_connection_.disconnect();
      ensenso_ptr_->closeDevices();
      ensenso_ptr_->closeTcpPort();
    }

    bool calibrateHandEyeCB(ensenso::CalibrateHandEye::Request& req, ensenso::CalibrateHandEye::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      // Check consistency between robot and pattern poses
      if ( req.robot_poses.poses.size() != ensenso_ptr_->getPatternCount() )
      {
        ROS_WARN("The number of robot_poses differs from the pattern count in the camera buffer");
        if (was_running)
          ensenso_ptr_->start();
        return true;
      }
      // Convert poses to Eigen::Affine3d
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > robot_eigen_list;
      for (size_t i = 0; i < req.robot_poses.poses.size(); i++) {
        Eigen::Affine3d pose;
        tf::poseMsgToEigen(req.robot_poses.poses[i], pose);
        robot_eigen_list.push_back(pose);
      }
      // Calibrate
      Eigen::Affine3d camera_seed, pattern_seed, estimated_camera_pose, estimated_pattern_pose;
      tf::poseMsgToEigen(req.camera_seed, camera_seed);
      tf::poseMsgToEigen(req.pattern_seed, pattern_seed);
      ROS_INFO("calibrateHandEye: It may take up to 5 minutes...");
      res.success = ensenso_ptr_->calibrateHandEye(robot_eigen_list, camera_seed, pattern_seed,
                      req.setup, estimated_camera_pose, estimated_pattern_pose, res.iterations,
                      res.reprojection_error);
      if (res.success)
      {
        ROS_INFO("Calibration computation finished");
        tf::poseEigenToMsg(estimated_camera_pose, res.estimated_camera_pose);
        tf::poseEigenToMsg(estimated_pattern_pose, res.estimated_pattern_pose);
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }

    void cameraParametersCallback(ensenso::CameraParametersConfig &config, uint32_t level)
    {
      // Process enumerators
      std::string trigger_mode, profile;
      switch (config.TriggerMode)
      {
        case 0:
          trigger_mode = "Software";
          break;
        case 1:
          trigger_mode = "FallingEdge";
          break;
        case 2:
          trigger_mode = "RisingEdge";
          break;
        default:
          trigger_mode = "Software";
      }
      switch (config.OptimizationProfile)
      {
        case 0:
          profile = "Aligned";
          break;
        case 1:
          profile = "Diagonal";
          break;
        case 2:
          profile = "AlignedAndDiagonal";
          break;
        default:
          profile = "AlignedAndDiagonal";
      }
      ROS_DEBUG("---");
      ROS_DEBUG("Capture Parameters");
      ROS_DEBUG_STREAM("AutoBlackLevel: "   << std::boolalpha << config.AutoBlackLevel);
      ROS_DEBUG_STREAM("AutoExposure: "     << std::boolalpha << config.AutoExposure);
      ROS_DEBUG_STREAM("AutoGain: "         << std::boolalpha << config.AutoGain);
      ROS_DEBUG_STREAM("Binning: "          << config.Binning);
      ROS_DEBUG_STREAM("BlackLevelOffset: " << config.BlackLevelOffset);
      ROS_DEBUG_STREAM("Exposure: "         << config.Exposure);
      ROS_DEBUG_STREAM("FlexView: "         << std::boolalpha << config.FlexView);
      ROS_DEBUG_STREAM("FlexViewImages: "   << config.FlexViewImages);
      ROS_DEBUG_STREAM("FrontLight: "       << std::boolalpha << config.FrontLight);
      ROS_DEBUG_STREAM("Gain: "             << config.Gain);
      ROS_DEBUG_STREAM("GainBoost: "        << std::boolalpha << config.GainBoost);
      ROS_DEBUG_STREAM("HardwareGamma: "    << std::boolalpha << config.HardwareGamma);
      ROS_DEBUG_STREAM("Hdr: "              << std::boolalpha << config.Hdr);
      ROS_DEBUG_STREAM("PixelClock: "       << config.PixelClock);
      ROS_DEBUG_STREAM("Projector: "        << std::boolalpha << config.Projector);
      ROS_DEBUG_STREAM("TargetBrightness: " << config.TargetBrightness);
      ROS_DEBUG_STREAM("TriggerMode: "      << trigger_mode);
      ROS_DEBUG_STREAM("RGBTriggerDelay: "      << config.RGBTriggerDelay);
      ROS_DEBUG_STREAM("DisparityMapAOI: "  << std::boolalpha << config.DisparityMapAOI);
      ROS_DEBUG("Stereo Matching Parameters");
      ROS_DEBUG_STREAM("MinimumDisparity: "     << config.MinimumDisparity);
      ROS_DEBUG_STREAM("NumberOfDisparities: "  << config.NumberOfDisparities);
      ROS_DEBUG_STREAM("OptimizationProfile: "  << profile);
      ROS_DEBUG_STREAM("Scaling: "              << config.Scaling);
      ROS_DEBUG("Advanced Matching Parameters");
      ROS_DEBUG_STREAM("DepthChangeCost: " << config.DepthChangeCost);
      ROS_DEBUG_STREAM("DepthStepCost: " << config.DepthStepCost);
      ROS_DEBUG_STREAM("ShadowingThreshold: " << config.ShadowingThreshold);
      ROS_DEBUG("Postprocessing Parameters");
      ROS_DEBUG_STREAM("Find Pattern: "   << std::boolalpha << config.FindPattern);
      if (!config.FindPattern)
      {
        ROS_WARN_STREAM("The calibration pattern will not be searched for, calibration will not work.");
      }
      ROS_DEBUG_STREAM("UniquenessRatio: " << config.UniquenessRatio);
      ROS_DEBUG_STREAM("MedianFilterRadius: "<< config.MedianFilterRadius);
      ROS_DEBUG_STREAM("SpeckleComponentThreshold: "<< config.SpeckleComponentThreshold);
      ROS_DEBUG_STREAM("SpeckleRegionSize: "<< config.SpeckleRegionSize);
      ROS_DEBUG_STREAM("FillBorderSpread: "<< config.FillBorderSpread);
      ROS_DEBUG_STREAM("FillRegionSize: " << config.FillRegionSize);
      ROS_DEBUG("Render Parameters");
      ROS_DEBUG_STREAM("SurfaceConnectivity: "   << std::boolalpha << config.SurfaceConnectivity);
      ROS_DEBUG_STREAM("NearPlane: "   << std::boolalpha << config.NearPlane);
      ROS_DEBUG_STREAM("FarPlane: "   << std::boolalpha << config.FarPlane);
      ROS_DEBUG_STREAM("UseOpenGL: "   << std::boolalpha << config.UseOpenGL);
      ROS_DEBUG("Stream Parameters