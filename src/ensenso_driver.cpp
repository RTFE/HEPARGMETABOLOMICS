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
      ROS_DEBUG("Stream Parameters");
      ROS_DEBUG_STREAM("Cloud: "   << std::boolalpha << config.Cloud);
      ROS_DEBUG_STREAM("Images: "   << std::boolalpha << config.Images);
      ROS_DEBUG_STREAM("Depth: " << std::boolalpha << config.Depth);
      ROS_DEBUG("CUDA Parameters");
      #ifdef CUDA_IMPLEMENTED
        ROS_DEBUG_STREAM("Use CUDA: "   << std::boolalpha << config.EnableCUDA);
      #else
        ROS_DEBUG_STREAM("CUDA is not supported. Upgrade EnsensoSDK to Version >= 2.1.7 in order to use CUDA.");
      #endif
      ROS_DEBUG("---");

      enable_cloud_ = config.Cloud;
      enable_images_ = config.Images;
      enable_depth_ = config.Depth;

      //advertise topics only when parameters are set accordingly
      if (config.FindPattern && !find_pattern_)
      {
        pattern_raw_pub_ = nh_.advertise<ensenso::RawStereoPattern> ("pattern/stereo", 1, false);
        pattern_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ("pattern/pose", 1, false);
        calibrate_srv_ = nh_.advertiseService("calibrate_handeye", &EnsensoDriver::calibrateHandEyeCB, this);
        pattern_srv_ = nh_.advertiseService("estimate_pattern_pose", &EnsensoDriver::estimatePatternPoseCB, this);
        collect_srv_ = nh_.advertiseService("collect_pattern", &EnsensoDriver::collectPatternCB, this);
      }
      find_pattern_ = config.FindPattern;
      // Capture parameters
      ensenso_ptr_->setAutoBlackLevel(config.AutoBlackLevel);
      ensenso_ptr_->setAutoExposure(config.AutoExposure);
      ensenso_ptr_->setAutoGain(config.AutoGain);
      ensenso_ptr_->setBlackLevelOffset(config.BlackLevelOffset);
      ensenso_ptr_->setExposure(config.Exposure);
      ensenso_ptr_->setFrontLight(config.FrontLight);
      ensenso_ptr_->setGain(config.Gain);
      ensenso_ptr_->setGainBoost(config.GainBoost);
      ensenso_ptr_->setHardwareGamma(config.HardwareGamma);
      ensenso_ptr_->setHdr(config.Hdr);
      ensenso_ptr_->setPixelClock(config.PixelClock);
      ensenso_ptr_->setProjector(config.Projector);
      ensenso_ptr_->setRGBTriggerDelay(config.RGBTriggerDelay);
      ensenso_ptr_->setTargetBrightness(config.TargetBrightness);
      ensenso_ptr_->setTriggerMode(trigger_mode);
      ensenso_ptr_->setUseDisparityMapAreaOfInterest(config.DisparityMapAOI);
      // Flexview and binning only work in 'Software' trigger mode and with the projector on
      if (trigger_mode.compare("Software") == 0 && config.Projector)
      {
        ensenso_ptr_->setBinning(config.Binning);
        ensenso_ptr_->setFlexView(config.FlexView, config.FlexViewImages);
      }
      // Stereo parameters
      ensenso_ptr_->setMinimumDisparity(config.MinimumDisparity);
      ensenso_ptr_->setNumberOfDisparities(config.NumberOfDisparities);
      ensenso_ptr_->setOptimizationProfile(profile);
      ensenso_ptr_->setScaling(config.Scaling);
      ensenso_ptr_->setDepthChangeCost(config.DepthChangeCost);
      ensenso_ptr_->setDepthStepCost(config.DepthStepCost);
      ensenso_ptr_->setShadowingThreshold(config.ShadowingThreshold);
      //Postprocessing parameters
      ensenso_ptr_->setUniquenessRatio(config.UniquenessRatio);
      ensenso_ptr_->setMedianFilterRadius(config.MedianFilterRadius);
      ensenso_ptr_->setSpeckleComponentThreshold(config.SpeckleComponentThreshold);
      ensenso_ptr_->setSpeckleRegionSize(config.SpeckleRegionSize);
      ensenso_ptr_->setFillBorderSpread(config.FillBorderSpread);
      ensenso_ptr_->setFillRegionSize(config.FillRegionSize);
      ensenso_ptr_->setFindPattern(config.FindPattern);
      //Render parameters
      ensenso_ptr_->setSurfaceConnectivity(config.SurfaceConnectivity);
      ensenso_ptr_->setNearPlane(config.NearPlane);
      ensenso_ptr_->setFarPlane(config.FarPlane);
      ensenso_ptr_->setUseOpenGL(config.UseOpenGL);
      //CUDA parameter
      #ifdef CUDA_IMPLEMENTED
        ensenso_ptr_->setEnableCUDA(config.EnableCUDA);
      #endif
      // Streaming parameters - only request rgb when available
      if (trigger_mode_ != config.TriggerMode)
      {
        trigger_mode_ = config.TriggerMode;
        if (ensenso_ptr_->isRunning())
        {
          ensenso_ptr_->stop();
          ensenso_ptr_->start();
        }
      }
      //check if someone is subscribed and start!
      depthSubscribeCallback();
      imagesSubscribeCallback();
      cloudSubscribeCallback();
    }

    bool collectPatternCB(ensenso::CollectPattern::Request& req, ensenso::CollectPattern::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      // Check consistency
      if (!req.decode && req.grid_spacing <= 0)
      {
        ROS_WARN("grid_spacing not specify. Forgot to set the request.decode = True?");
        if (was_running)
          ensenso_ptr_->start();
        return true;
      }
      // Discard previously saved patterns
      if (req.clear_buffer)
        ensenso_ptr_->discardPatterns();
      // Set the grid spacing
      if (req.decode)
      {
        res.grid_spacing = ensenso_ptr_->decodePattern();
        // Check consistency
        if (res.grid_spacing <= 0)
        {
          ROS_WARN("Couldn't decode calibration pattern");
          if (was_running)
            ensenso_ptr_->start();
          return true;
        }
      }
      else
        res.grid_spacing = req.grid_spacing;
      ensenso_ptr_->setGridSpacing(res.grid_spacing);
      // Collect pattern
      int prev_pattern_count = ensenso_ptr_->getPatternCount();
      res.pattern_count = ensenso_ptr_->collectPattern(req.add_to_buffer);
      res.success = (res.pattern_count == prev_pattern_count+1);
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }

    bool estimatePatternPoseCB(ensenso::EstimatePatternPose::Request& req, ensenso::EstimatePatternPose::Response &res)
    {
      bool was_running = ensenso_ptr_->isRunning();
      if (was_running)
        ensenso_ptr_->stop();
      res.success = ensenso_ptr_->getPatternCount() > 0;
      if (res.success)
      {
        Eigen::Affine3d pattern_pose;
        res.success = ensenso_ptr_->estimatePatternPose(pattern_pose, req.average);
        tf::poseEigenToMsg(pattern_pose, res.pose);
      }
      if (was_running)
        ensenso_ptr_->start();
      return true;
    }

    void cloudCallback( const boost::shared_ptr<PointCloudXYZ>& cloud)
    {
      // Point cloud
      if (cloud_pub_.getNumSubscribers() > 0)
      {
        ros::Time stamp;
        //stamp is the same for all images/cloud
        pcl_conversions::fromPCL(cloud->header.stamp, stamp);
        cloud->header.frame_id = camera_frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = stamp;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_pub_.publish(cloud_msg);
      }
    }

    void cloudRGBCallback( const boost::shared_ptr<PointCloudXYZRGBA>& cloud)
    {
      // Point cloud
      if (cloud_pub_.getNumSubscribers() > 0)
      {
        ros::Time stamp;
        //stamp is the same for all images/cloud
        pcl_conversions::fromPCL(cloud->header.stamp, stamp);
        cloud->header.frame_id = rgb_camera_frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = stamp;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_pub_.publish(cloud_msg);
      }
    }

    void imagesCallback( const boost::shared_ptr<PairOfImages>& rawimages, const boost::shared_ptr<PairOfImages>& rectifiedimages)
    {
      ros::Time stamp;
      //stamp is the same for all images
      pcl_conversions::fromPCL(rawimages->first.header.stamp, stamp);
      // Get cameras info
      sensor_msgs::CameraInfo linfo, rinfo;
      ensenso_ptr_->getCameraInfo("Left", linfo);
      ensenso_ptr_->getCameraInfo("Right", rinfo);
      linfo.header.stamp = stamp;
      linfo.header.frame_id = camera_frame_id_;
      rinfo.header.stamp = stamp;
      rinfo.header.frame_id = camera_frame_id_;
      // Images
      if (l_raw_pub_.getNumSubscribers() > 0)
        l_raw_pub_.publish(*toImageMsg(rawimages->first, stamp, camera_frame_id_), linfo, stamp);
      if (r_raw_pub_.getNumSubscribers() > 0)
        r_raw_pub_.publish(*toImageMsg(rawimages->second, stamp, camera_frame_id_), rinfo, stamp);
      if (l_rectified_pub_.getNumSubscribers() > 0)
        l_rectified_pub_.publish(toImageMsg(rectifiedimages->first, stamp, camera_frame_id_));
      if (r_rectified_pub_.getNumSubscribers() > 0)
        r_rectified_pub_.publish(toImageMsg(rectifiedimages->second, stamp, camera_frame_id_));
      // Publish calibration pattern info (if any)
      publishCalibrationPattern(stamp);
    }

    void imagesRGBCallback( const boost::shared_ptr<PairOfImages>& rawimages, const boost::shared_ptr<PairOfImages>& rectifiedimages,
                            const boost::shared_ptr<PairOfImages>& rgbimages)
    {
      ros::Time stamp;
      //stamp is the same for all images/cloud
      pcl_conversions::fromPCL(rawimages->first.header.stamp, stamp);
      // Get cameras info
      sensor_msgs::CameraInfo linfo, rinfo, rgbinfo;
      ensenso_ptr_->getCameraInfo("Left", linfo);
      ensenso_ptr_->getCameraInfo("Right", rinfo);
      ensenso_ptr_->getCameraInfo("RGB", rgbinfo);
      linfo.header.stamp = stamp;
      linfo.header.frame_id = camera_frame_id_;
      rinfo.header.stamp = stamp;
      rinfo.header.frame_id = camera_frame_id_;
      rgbinfo.header.stamp = stamp;
      rgbinfo.header.frame_id = rgb_camera_frame_id_;
      // Images
      if (l_raw_pub_.getNumSubscribers() > 0)
        l_raw_pub_.publish(*toImageMsg(rawimages->first, stamp, camera_frame_id_), linfo, stamp);
      if (r_raw_pub_.getNumSubscribers() > 0)
        r_raw_pub_.publish(*toImageMsg(rawimages->second, stamp, camera_frame_id_), rinfo, stamp);
      if (l_rectified_pub_.getNumSubscribers() > 0)
        l_rectified_pub_.publish(toImageMsg(rectifiedimages->first, stamp, camera_frame_id_));
      if (r_rectified_pub_.getNumSubscribers() > 0)
        r_rectified_pub_.publish(toImageMsg(rectifiedimages->second, stamp, camera_frame_id_));
      if (rgb_raw_pub_.getNumSubscribers() > 0)
        rgb_raw_pub_.publish(*toImageMsg(rgbimages->first, stamp, rgb_camera_frame_id_), rgbinfo, stamp);
      if (rgb_rectified_pub_.getNumSubscribers() > 0)
        rgb_rectified_pub_.publish(toImageMsg(rgbimages->second, stamp, rgb_camera_frame_id_));
      // Publish calibration pattern info (if any)
      publishCalibrationPattern(stamp);
    }

    void depthCallback( const boost::shared_ptr<pcl::PCLGenImage<float> >& depthimage)
    {
      if (depth_pub_.getNumSubscribers() > 0)
      {
        std::string frame_id = rgb_available_ ? rgb_camera_frame_id_ : camera_frame_id_;

        ros::Time stamp;
        //stamp is the same for all images/cloud
        pcl_conversions::fromPCL(depthimage->header.stamp, stamp);
        sensor_msgs::CameraInfo dinfo;
        ensenso_ptr_->getCameraInfo("Depth", dinfo);
        dinfo.header.stamp = stamp;
        dinfo.header.frame_id = frame_id;
        depth_pub_.publish(*toImageMsg(*depthimage, stamp, frame_id), dinfo, stamp);
      }
    }

    void publishTF()
    {
      pcl::Transform ensenso_tf;
      if (!ensenso_ptr_->getTFLeftToRGB(ensenso_tf))
      {
        return;
      }
      geometry_msgs::TransformStamped tf;
      tf.header.frame_id = camera_frame_id_;
      tf.header.stamp = ros::Time::now();
      tf.child_frame_id = rgb_camera_frame_id_;
      tf.transform.rotation.x = ensenso_tf.qx;
      tf.transform.rotation.y = ensenso_tf.qy;
      tf.transform.rotation.z = ensenso_tf.qz;
      tf.transform.rotation.w = ensenso_tf.qw;
      tf.transform.translation.x = ensenso_tf.tx;
      tf.transform.translation