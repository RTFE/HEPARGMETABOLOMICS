
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