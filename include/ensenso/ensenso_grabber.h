
#ifndef __ENSENSO_ENSENSO_GRABBER__
#define __ENSENSO_ENSENSO_GRABBER__

// PCL
#include <pcl/pcl_config.h>
#include <pcl/common/io.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/common/synchronizer.h>
// Others
#include <Eigen/Geometry>
#include <boost/thread.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
// Ensenso SDK
#include <nxLib.h>
namespace pcl
{
template <typename T>
struct PCLGenImage : PCLImage
{
    std::vector<T> data;
    typedef boost::shared_ptr< ::pcl::PCLGenImage<T> > Ptr;
    typedef boost::shared_ptr< ::pcl::PCLGenImage<T>  const> ConstPtr;
};

struct Transform
{
    double tx, ty, tz, qx, qy, qz, qw;
};
/**
 * @brief Grabber for IDS-Imaging Ensenso's devices
 * @author Francisco Suarez-Ruiz
 */
class PCL_EXPORTS EnsensoGrabber : public Grabber
{
    typedef std::pair<pcl::PCLGenImage<pcl::uint8_t>, pcl::PCLGenImage<pcl::uint8_t> > PairOfImages;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** @cond */
    typedef boost::shared_ptr<EnsensoGrabber> Ptr;
    typedef boost::shared_ptr<const EnsensoGrabber> ConstPtr;

    // Define callback signature typedefs
    typedef void
    (sig_cb_ensenso_point_cloud)(const pcl::PointCloud<pcl::PointXYZ>::Ptr &);

    typedef void
    (sig_cb_ensenso_point_cloud_rgb)(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &);

    typedef void
    (sig_cb_ensenso_images)(const boost::shared_ptr<PairOfImages>&, const boost::shared_ptr<PairOfImages>&);

    typedef void
    (sig_cb_ensenso_images_rgb)(const boost::shared_ptr<PairOfImages>&, const boost::shared_ptr<PairOfImages>&, const boost::shared_ptr<PairOfImages>&);

    typedef void
    (sig_cb_ensenso_image_depth)(const boost::shared_ptr<pcl::PCLGenImage<float> >&);

    /** @endcond */

    /** @brief Constructor */
    EnsensoGrabber ();

    /** @brief Destructor inherited from the Grabber interface. It never throws. */
    virtual ~EnsensoGrabber () throw ();

    /** @brief With this command you can calibrate the position of the camera with respect to a robot.
    * The calibration routine currently supports two types of setups: either your camera is fixed with
    * respect to the robot origin, or your camera mounted on the robot hand and is moving with the robot.
    * @param[in] robot_poses A list of robot poses, 1 for each pattern acquired (in the same order)