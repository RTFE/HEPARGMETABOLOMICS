
#include "ensenso/ensenso_grabber.h"
#include <boost/make_shared.hpp>

void ensensoExceptionHandling (const NxLibException &ex,
                 std::string func_nam)
{
  PCL_ERROR ("%s: NxLib error %s (%d) occurred while accessing item %s.\n", func_nam.c_str (), ex.getErrorText ().c_str (), ex.getErrorCode (),
         ex.getItemPath ().c_str ());
  if (ex.getErrorCode () == NxLibExecutionFailed)
  {
    NxLibCommand cmd ("");
    PCL_WARN ("\n%s\n", cmd.result ().asJson (true, 4, false).c_str ());
  }
}


pcl::EnsensoGrabber::EnsensoGrabber () :
  device_open_ (false),
  mono_device_open_(false),
  last_stereo_pattern_(""),
  store_calibration_pattern_ (false),
  running_ (false),
  tcp_open_ (false),
  use_rgb_(false)
{
  point_cloud_signal_ = createSignal<sig_cb_ensenso_point_cloud> ();
  point_cloud_rgb_signal_ = createSignal<sig_cb_ensenso_point_cloud_rgb> ();
  images_signal_ = createSignal<sig_cb_ensenso_images> ();
  images_rgb_signal_ = createSignal<sig_cb_ensenso_images_rgb> ();
  image_depth_signal_ = createSignal<sig_cb_ensenso_image_depth> ();

  PCL_INFO ("Initialising nxLib\n");
  try
  {
    nxLibInitialize ();
    root_.reset (new NxLibItem);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "EnsensoGrabber");
    PCL_THROW_EXCEPTION (pcl::IOException, "Could not initialise NxLib.");  // If constructor fails; throw exception
  }
}

pcl::EnsensoGrabber::~EnsensoGrabber () throw ()
{
  try
  {
    stop ();
    root_.reset ();

    disconnect_all_slots<sig_cb_ensenso_point_cloud> ();
    disconnect_all_slots<sig_cb_ensenso_point_cloud_rgb> ();
    disconnect_all_slots<sig_cb_ensenso_images> ();
    disconnect_all_slots<sig_cb_ensenso_images_rgb> ();
    disconnect_all_slots<sig_cb_ensenso_image_depth> ();

    if (tcp_open_)
      closeTcpPort ();
    nxLibFinalize ();
  }
  catch (...)
  {
    // destructor never throws
  }
}

bool pcl::EnsensoGrabber::calibrateHandEye (const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &robot_poses,
                                            const Eigen::Affine3d &camera_seed,
                                            const Eigen::Affine3d &pattern_seed,
                                            const std::string setup,
                                            Eigen::Affine3d &estimated_camera_pose,
                                            Eigen::Affine3d &estimated_pattern_pose,
                                            int &iterations,
                                            double &reprojection_error) const
{
  if ( (*root_)[itmVersion][itmMajor] <= 1 && (*root_)[itmVersion][itmMinor] < 3)
  {
    PCL_WARN("EnsensoSDK 1.3.x fixes bugs into calibration optimization\n");
    PCL_WARN("please update your SDK! http://www.ensenso.de/support/sdk-download\n");
  }
  NxLibCommand calibrate (cmdCalibrateHandEye);
  try
  {
    // Check consistency
    if (!boost::iequals (setup, valFixed) && !boost::iequals (setup, valMoving))
    {
      PCL_WARN("Received invalid setup value: %s\n", setup.c_str());
      return (false);
    }
    // Set Hand-Eye calibration parameters
    PCL_DEBUG("Setting calibration parameters\n");
    std::string target;
    if (boost::iequals (setup, valFixed))
      target = valWorkspace;
    else
      target = valHand;
    Eigen::Affine3d eigen_pose;
    // Feed robot transformations
    PCL_DEBUG("Populating robot poses\n");
    std::vector<std::string> json_poses;
    json_poses.resize (robot_poses.size ());
    for (uint i = 0; i < robot_poses.size(); ++i)
    {
      eigen_pose = robot_poses[i];
      eigen_pose.translation () *= 1000.0; // meters -> millimeters
      matrixToJson(eigen_pose, json_poses[i]);
    }
    PCL_DEBUG("Executing...\n");
    // Convert camera seed to Json
    std::string json_camera_seed, json_pattern_seed;
    PCL_DEBUG("Populating seeds\n");
    eigen_pose = camera_seed;
    eigen_pose.translation () *= 1000.0; // meters -> millimeters
    matrixToJson(eigen_pose, json_camera_seed);
    PCL_DEBUG("camera_seed:\n %s\n", json_camera_seed.c_str());
    // Convert pattern seed to Json
    eigen_pose = pattern_seed;
    eigen_pose.translation () *= 1000.0; // meters -> millimeters
    matrixToJson(pattern_seed, json_pattern_seed);
    PCL_DEBUG("pattern_seed:\n %s\n", json_pattern_seed.c_str());
    // Populate command parameters
    // It's very important to write the parameters in alphabetical order and at the same time!
    calibrate.parameters ()[itmLink].setJson(json_camera_seed, false);
    calibrate.parameters ()[itmPatternPose].setJson(json_pattern_seed, false);
    calibrate.parameters ()[itmSetup] = setup;
    calibrate.parameters ()[itmTarget] = target;
    for (uint i = 0; i < json_poses.size(); ++i)
      calibrate.parameters ()[itmTransformations][i].setJson(json_poses[i], false);
    // Execute the command
    calibrate.execute ();  // It might take some minutes
    if (calibrate.successful())
    {
      // It's very important to read the parameters in alphabetical order and at the same time!
      iterations = calibrate.result()[itmIterations].asInt();
      std::string json_camera_pose = calibrate.result()[itmLink].asJson (true);
      std::string json_pattern_pose = calibrate.result()[itmPatternPose].asJson (true);
      reprojection_error = calibrate.result()[itmReprojectionError].asDouble();
      // Estimated camera pose
      jsonToMatrix(json_camera_pose, estimated_camera_pose);
      estimated_camera_pose.translation () /= 1000.0; // millimeters -> meters
      // Estimated pattern pose
      jsonToMatrix(json_pattern_pose, estimated_pattern_pose);
      estimated_pattern_pose.translation () /= 1000.0; // millimeters -> meters
      PCL_DEBUG("Result:\n %s\n", json_camera_pose.c_str());
      return (true);
    }
    else
      return (false);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "calibrateHandEye");
    return (false);
  }
}

bool pcl::EnsensoGrabber::closeDevices ()
{
  if (!device_open_ && !mono_device_open_)
    return (false);

  stop ();
  PCL_INFO ("Closing Ensenso cameras\n");

  try
  {
    NxLibCommand (cmdClose).execute ();
    device_open_ = false;
    mono_device_open_ = false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeDevice");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::closeTcpPort ()
{
  try
  {
    nxLibCloseTcpPort ();
    tcp_open_ = false;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "closeTcpPort");
    return (false);
  }
  return (true);
}

int pcl::EnsensoGrabber::collectPattern (const bool buffer) const
{
  if (!device_open_ || running_)
    return (-1);
  try
  {
    NxLibCommand (cmdCapture).execute ();
    NxLibCommand collect_pattern (cmdCollectPattern);
    collect_pattern.parameters ()[itmBuffer].set (buffer);
    collect_pattern.parameters ()[itmDecodeData].set (false);
    collect_pattern.execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "collectPattern");
    return getPatternCount();
  }
  return getPatternCount();
}

double pcl::EnsensoGrabber::decodePattern () const
{
  double grid_spacing = -1.0;
  if (!device_open_ || running_)
    return (-1.0);
  try
  {
    NxLibCommand (cmdCapture).execute ();
    NxLibCommand collect_pattern (cmdCollectPattern);
    collect_pattern.parameters ()[itmBuffer].set (false);
    collect_pattern.parameters ()[itmDecodeData].set (true);
    collect_pattern.execute ();
    grid_spacing = collect_pattern.result()[itmGridSpacing].asDouble();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "decodePattern");
    return (-1.0);
  }
  return grid_spacing;
}

bool pcl::EnsensoGrabber::discardPatterns () const
{
  try
  {
    NxLibCommand (cmdDiscardPatterns).execute ();
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "discardPatterns");
    return (false);
  }
  return (true);
}

bool pcl::EnsensoGrabber::estimatePatternPose (Eigen::Affine3d &pose, const bool average) const
{
  try
  {
    NxLibCommand estimate_pattern_pose (cmdEstimatePatternPose);
    estimate_pattern_pose.parameters ()[itmAverage].set (average);
    estimate_pattern_pose.execute ();
    NxLibItem tf = estimate_pattern_pose.result ()[itmPatternPose];
    // Convert tf into a matrix
    if (!jsonToMatrix (tf.asJson (), pose))
      return (false);
    pose.translation () /= 1000.0;  // Convert translation in meters (Ensenso API returns milimeters)
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "estimateCalibrationPatternPoses");
    return (false);
  }
}

int pcl::EnsensoGrabber::enumDevices () const
{
  int camera_count = 0;
  try
  {
    NxLibItem cams = NxLibItem ("/Cameras/BySerialNo");
    camera_count = cams.count ();
    // Print information for all cameras in the tree
    PCL_INFO ("Number of connected cameras: %d\n", camera_count);
    PCL_INFO ("Serial No    Model   Status\n");
    for (int n = 0; n < cams.count (); ++n)
    {
      PCL_INFO ("%s   %s   %s\n", cams[n][itmSerialNumber].asString ().c_str (),
            cams[n][itmModelName].asString ().c_str (),
            cams[n][itmStatus].asString ().c_str ());
    }
    PCL_INFO ("\n");
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "enumDevices");
  }
  return (camera_count);
}

bool pcl::EnsensoGrabber::getCameraInfo(std::string cam, sensor_msgs::CameraInfo &cam_info) const
{
  try
  {
    bool depth = false;
    if (cam == "Depth")
    {
      depth = true;
      cam  = use_rgb_ ? "RGB" : "Left";
    }
    NxLibItem camera = (cam == "RGB" ) ? monocam_ : camera_;
    NxLibItem camera_mat = (cam == "RGB") ? monocam_[itmCalibration][itmCamera] : camera_[itmCalibration][itmDynamic][itmStereo][cam][itmCamera];
    NxLibItem camera_dist = (cam == "RGB") ? monocam_[itmCalibration][itmDistortion] : camera_[itmCalibration][itmMonocular][cam][itmDistortion];

    cam_info.width = camera[itmSensor][itmSize][0].asInt();
    cam_info.height = camera[itmSensor][itmSize][1].asInt();
    cam_info.distortion_model = "plumb_bob";
    // Distorsion factors (as in ROS CameraInfo Documentation, [K1, K2, T1, T2, K3])

    cam_info.D.resize(5);
    if (depth)
    {
      for(std::size_t i = 0; i < cam_info.D.size(); ++i)
      {
          cam_info.D[i] = 0;
      }
    }
    else
    {
      cam_info.D[0] = camera_dist[0].asDouble();
      cam_info.D[1] = camera_dist[1].asDouble();
      cam_info.D[2] = camera_dist[5].asDouble();
      cam_info.D[3] = camera_dist[6].asDouble();
      cam_info.D[4] = camera_dist[2].asDouble();
    }

    // K and R matrices
    for(std::size_t i = 0; i < 3; ++i)
    {
      for(std::size_t j = 0; j < 3; ++j)
      {
        if (cam != "RGB")
        {
          cam_info.R[3*i+j] = camera[itmCalibration][itmDynamic][itmStereo][cam][itmRotation][j][i].asDouble();
          cam_info.K[3*i+j] = camera_[itmCalibration][itmMonocular][cam][itmCamera][j][i].asDouble();
        }
        else
        {
          cam_info.K[3*i+j] = camera_mat[j][i].asDouble();
        }
      }
    }
    if (cam == "RGB")
    {
      cam_info.R[0] = 1.0;
      cam_info.R[4] = 1.0;
      cam_info.R[8] = 1.0;
    }
    //first row
    cam_info.P[0] = camera_mat[0][0].asDouble();
    cam_info.P[1] = camera_mat[1][0].asDouble();
    cam_info.P[2] = camera_mat[2][0].asDouble();
    cam_info.P[3] = 0.0;
    //second row
    cam_info.P[4] = camera_mat[0][1].asDouble();
    cam_info.P[5] = camera_mat[1][1].asDouble();
    cam_info.P[6] = camera_mat[2][1].asDouble();
    cam_info.P[7] = 0.0;
    //third row
    cam_info.P[8] = 0.0;
    cam_info.P[9] = 0.0;
    cam_info.P[10] = 1.0;
    cam_info.P[11] = 0.0;
    if (cam == "Right")
    {
      double B = camera[itmCalibration][itmStereo][itmBaseline].asDouble() / 1000.0;
      double fx = cam_info.P[0];
      cam_info.P[3] = (-fx * B);
    }
    return true;
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "getCameraInfo");
    return false;
  }
}

bool pcl::EnsensoGrabber::getTFLeftToRGB(Transform& tf) const
{
  if (!mono_device_open_)
  {
    return (false);
  }
  //get Translation between left camera and RGB frame
  try {
    NxLibCommand convert(cmdConvertTransformation, "convert");
    convert.parameters()[itmTransformation].setJson(monocam_[itmLink].asJson());
    convert.execute();
    Eigen::Affine3d transform;
    for (int i = 0; i < 4 ; ++i)
    {
      for (int j = 0; j < 4 ; ++j)
      {
        transform(j,i) = convert.result()[itmTransformation][i][j].asDouble();
      }
    }
    Eigen::Affine3d inv_transform = transform.inverse();
    Eigen::Quaterniond q(inv_transform.rotation()); //from stereo to rgb
    q.normalize();
    tf.qx = q.x();
    tf.qy = q.y();
    tf.qz = q.z();
    tf.qw = q.w();
    tf.tx = inv_transform.translation().x() / 1000.0;
    tf.ty = inv_transform.translation().y() / 1000.0;
    tf.tz = inv_transform.translation().z() / 1000.0;
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "setUseRGB");
    return (false);
  }
  return (false);
}


bool pcl::EnsensoGrabber::getLastCalibrationPattern ( std::vector<int> &grid_size, double &grid_spacing,
                                                      std::vector<Eigen::Vector2d> &left_points,
                                                      std::vector<Eigen::Vector2d> &right_points,
                                                      Eigen::Affine3d &pose) const
{
  // Lock
  pattern_mutex_.lock ();
  std::string stereo_pattern_json = last_stereo_pattern_;
  pose = last_pattern_pose_;
  pattern_mutex_.unlock ();
  // Process
  if ( stereo_pattern_json.empty() )
    return false;
  // Use NxLib JSON API to extract the Raw Stereo Pattern information
  NxLibItem stereo_pattern ("/tmpStereoPattern");
  stereo_pattern.setJson(stereo_pattern_json);
  // Get pattern details
  grid_size.resize(2);
  grid_size[0] = stereo_pattern[itmPattern][itmGridSize][0].asInt();
  grid_size[1] = stereo_pattern[itmPattern][itmGridSize][1].asInt();
  grid_spacing = stereo_pattern[itmPattern][itmGridSpacing].asDouble();
  int rows = grid_size[0];
  int cols = grid_size[1];
  left_points.resize(rows*cols);
  right_points.resize(rows*cols);
  for (uint i = 0; i < left_points.size(); ++i)
  {
    left_points[i][0] = stereo_pattern[itmPoints][0][i][0].asDouble();
    left_points[i][1] = stereo_pattern[itmPoints][0][i][1].asDouble();
    right_points[i][0] = stereo_pattern[itmPoints][1][i][0].asDouble();
    right_points[i][1] = stereo_pattern[itmPoints][1][i][1].asDouble();
  }
  stereo_pattern.erase();
  return true;
}

float pcl::EnsensoGrabber::getFramesPerSecond () const
{
  boost::mutex::scoped_lock lock (fps_mutex_);
  return fps_;
}

std::string pcl::EnsensoGrabber::getName () const
{
  return ("EnsensoGrabber");
}

std::string pcl::EnsensoGrabber::getOpenCVType (const int channels,
                        const int bpe,
                        const bool isFlt)
{
  int bits = bpe * 8;
  char type = isFlt ? 'F' : (bpe > 3 ? 'S' : 'U');
  return (boost::str (boost::format ("CV_%i%cC%i") % bits % type % channels));
}

pcl::uint64_t pcl::EnsensoGrabber::getPCLStamp (const double ensenso_stamp)
{
#if defined _WIN32 || defined _WIN64
  return (ensenso_stamp * 1000000.0);
#else
  return ( (ensenso_stamp - 11644473600.0) * 1000000.0);
#endif
}

int pcl::EnsensoGrabber::getPatternCount () const
{
  return ( (*root_)[itmParameters][itmPatternCount].asInt ());
}

bool pcl::EnsensoGrabber::grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  if (!device_open_)
    return (false);

  if (running_)
    return (false);

  try
  {
    NxLibCommand (cmdCapture).execute ();
    // Stereo matching task
    NxLibCommand (cmdComputeDisparityMap).execute ();
    // Convert disparity map into XYZ data for each pixel
    NxLibCommand (cmdComputePointMap).execute ();
    // Get info about the computed point map and copy it into a std::vector
    double timestamp;
    std::vector<float> pointMap;
    int width, height;
    camera_[itmImages][itmRaw][itmLeft].getBinaryDataInfo (0, 0, 0, 0, 0, &timestamp);  // Get raw image timestamp
    camera_[itmImages][itmPointMap].getBinaryDataInfo (&width, &height, 0, 0, 0, 0);
    camera_[itmImages][itmPointMap].getBinaryData (pointMap, 0);
    // Copy point cloud and convert in meters
    cloud.header.stamp = getPCLStamp (timestamp);
    cloud.resize (height * width);
    cloud.width = width;
    cloud.height = height;
    cloud.is_dense = false;
    // Copy data in point cloud (and convert milimeters in meters)
    for (size_t i = 0; i < pointMap.size (); i += 3)
    {
      cloud.points[i / 3].x = pointMap[i] / 1000.0;
      cloud.points[i / 3].y = pointMap[i + 1] / 1000.0;
      cloud.points[i / 3].z = pointMap[i + 2] / 1000.0;
    }
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "grabSingleCloud");
    return (false);
  }
}

bool pcl::EnsensoGrabber::isRunning () const
{
  return (running_);
}

bool pcl::EnsensoGrabber::isTcpPortOpen () const
{
  return (tcp_open_);
}

bool pcl::EnsensoGrabber::jsonToMatrix (const std::string json, Eigen::Affine3d &matrix) const
{
  try
  {
    NxLibCommand convert (cmdConvertTransformation);
    convert.parameters ()[itmTransformation].setJson (json);
    convert.execute ();
    Eigen::Affine3d tmp (Eigen::Affine3d::Identity ());
    // Rotation
    tmp.linear ().col (0) = Eigen::Vector3d (convert.result ()[itmTransformation][0][0].asDouble (),
                         convert.result ()[itmTransformation][0][1].asDouble (),
                         convert.result ()[itmTransformation][0][2].asDouble ());
    tmp.linear ().col (1) = Eigen::Vector3d (convert.result ()[itmTransformation][1][0].asDouble (),
                         convert.result ()[itmTransformation][1][1].asDouble (),
                         convert.result ()[itmTransformation][1][2].asDouble ());
    tmp.linear ().col (2) = Eigen::Vector3d (convert.result ()[itmTransformation][2][0].asDouble (),
                         convert.result ()[itmTransformation][2][1].asDouble (),
                         convert.result ()[itmTransformation][2][2].asDouble ());
    // Translation
    tmp.translation () = Eigen::Vector3d (convert.result ()[itmTransformation][3][0].asDouble (),
                        convert.result ()[itmTransformation][3][1].asDouble (),
                        convert.result ()[itmTransformation][3][2].asDouble ());
    matrix = tmp;
    return (true);
  }
  catch (NxLibException &ex)
  {
    ensensoExceptionHandling (ex, "jsonToMatrix");
    return (false);
  }
}

bool pcl::EnsensoGrabber::matrixToJson (const Eigen::Affine3d &matrix, std::string &json,
                                        const bool pretty_format) const
{
  try
  {
    NxLibCommand convert (cmdConvertTransformation);
    // Rotation
    convert.parameters ()[itmTransformation][0][0].set (matrix.linear ().col (0)[0]);
    convert.parameters ()[itmTransformation][0][1].set (matrix.linear ().col (0)[1]);
    convert.parameters ()[itmTransformation][0][2].set (matrix.linear ().col (0)[2]);
    convert.parameters ()[itmTransformation][0][3].set (0.0);
    convert.parameters ()[itmTransformation][1][0].set (matrix.linear ().col (1)[0]);
    convert.parameters ()[itmTransformation][1][1].set (matrix.linear ().col (1)[1]);
    convert.parameters ()[itmTransformation][1][2].set (matrix.linear ().col (1)[2]);
    convert.parameters ()[itmTransformation][1][3].set (0.0);
    convert.parameters ()[itmTransformation][2][0].set (matrix.linear ().col (2)[0]);
    convert.parameters ()[itmTransformation][2][1].set (matrix.linear ().col (2)[1]);
    convert.parameters ()[itmTransformation][2][2].set (matrix.linear ().col (2)[2]);
    convert.parameters ()[itmTransformation][2][3].set (0.0);
    // Translation
    convert.parameters ()[itmTransformation][3][0].set (matrix.translation ()[0]);
    convert.parameters ()[itmTransformation][3][1].set (matrix.translation ()[1]);
    convert.parameters ()[itmTransformation][3][2].set (matrix.translation ()[2]);