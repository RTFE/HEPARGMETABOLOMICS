
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