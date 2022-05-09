
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
    * @param[in] camera_seed Initial guess of the camera pose. The pose must be given relative to the
    * robot hand (for a moving camera setup), or relative to the robot origin (for the fixed camera setup).
    * @param[in] pattern_seed Initial guess of the pattern pose. This pose must be given relative to the
    * robot hand (for a fixed camera setup), or relative to the robot origin (for the moving camera setup).
    * @param[in] setup Moving or Fixed, please refer to the Ensenso documentation
    * @param[out] estimated_camera_pose The Transformation between this camera's left eye coordinates and
    * the next linked system.
    * @param[out] estimated_pattern_pose The estimated pattern pose. This pose is either relative to the
    * robot hand (for a fixed camera setup), or relative to the robot origin (for the moving camera setup).
    * @param[out] iterations Indicates the number of optimization iterations performed on the final solution
    * until it had converged.
    * @param[out] reprojection_error The reprojection error per pattern point over all collected patterns of
    * the final solution.
    * @return True if successful, false otherwise
    * @warning This can take up to 120 seconds */
    bool calibrateHandEye ( const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &robot_poses,
                            const Eigen::Affine3d &camera_seed,
                            const Eigen::Affine3d &pattern_seed,
                            const std::string setup,
                            Eigen::Affine3d &estimated_camera_pose,
                            Eigen::Affine3d &estimated_pattern_pose,
                            int &iterations,
                            double &reprojection_error) const;

    /** @brief Closes all Ensenso devices
     * @return True if successful, false otherwise */
    bool closeDevices ();

    /** @brief Close TCP port program
     * @return True if successful, false otherwise
     * @warning If you do not close the TCP port the program might exit with the port still open, if it is the case
     * use @code ps -ef @endcode and @code kill PID @endcode to kill the application and effectively close the port. */
    bool closeTcpPort (void);

    /** @brief Collects a calibration pattern
     * @param[in] buffer Specifies whether the pattern should be added to the pattern buffer.
     * @return the number of calibration patterns stored in the @code nxTree @endcode , -1 on error
     * @warning A device must be opened and must not be running.*/
    int collectPattern (const bool buffer=true) const;

    /** @brief Decodes the pattern grid size and thickness
     * @return the grid size in mm*/
    double decodePattern () const;

    /** @brief Clears the pattern buffers of monocular and stereo pattern observations.
     * @return True if successful, false otherwise
     * @note The counters PatternCount and MonocularPatternCount will be zero after clearing.*/
    bool discardPatterns () const;

    /** @brief Searches for available devices
     * @returns The number of Ensenso devices connected */
    int enumDevices () const;

    /** @brief Estimate the calibration pattern pose
     * @param[out] pose the calibration pattern pose
     * @param[in] average Specifies if all pattern point coordinates in the buffer
     * should be averaged to produce a more precise pose measurement. This will only
     * produce a correct result if all patterns in the buffer originate from
     * multiple images of the same pattern in the same pose.
     * @return true if successful, false otherwise
     * @warning A device must be opened and must not be running.
     * @note At least one calibration pattern must have been collected before, use collectPattern() before */
    bool estimatePatternPose (Eigen::Affine3d &pose, const bool average=false) const;

    /** @brief Get class name
     * @returns A string containing the class name */
    std::string getName () const;

    /** @brief Get meta information for a monocular camera.
     * @param[in] cam A string containing the camera (Left or Right)
     * @param[out] cam_info meta information for a camera.
     * @return True if successful, false otherwise
     * @note See: [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
     */
    bool getCameraInfo(std::string cam, sensor_msgs::CameraInfo &cam_info) const;

    /** @brief Get transformation between stereo frame and rgb frame.

     * @return True if successful, false otherwise
     */
    bool getTFLeftToRGB(Transform& tf) const;

    /** @brief Get the raw stereo pattern information and the pattern pose. Before using it enable the
     * storeCalibrationPattern.
     * @param[out] grid_size The number of points on the patterns grid as two element vector.
     * @param[out] grid_spacing Distance of two neighboring grid points along this pattern's x or y axis.
     * @param[out] left_points the raw image positions of the pattern dots in the first camera.
     * @param[out] right_points the raw image dot positions in the second camera.
     * @param[out] pose the calibration pattern pose.
     * @return True if successful, false otherwise */
    bool getLastCalibrationPattern (std::vector<int> &grid_size, double &grid_spacing,
                                    std::vector<Eigen::Vector2d> &left_points,
                                    std::vector<Eigen::Vector2d> &right_points,
                                    Eigen::Affine3d &pose) const;

    /** @brief Obtain the number of frames per second (FPS) */
    float getFramesPerSecond () const;

    /** @brief Gets the number of collected patterns with successful observations in two cameras.
     * @returns The number of pattern in the camera buffer */
    int getPatternCount () const;

    /** @brief Capture a single point cloud and store it
     * @param[out] cloud The cloud to be filled
     * @return True if successful, false otherwise
     * @warning A device must be opened and not running */
    bool grabSingleCloud (pcl::PointCloud<pcl::PointXYZ> &cloud);

    /** @brief Check if the data acquisition is still running
     * @return True if running, false otherwise */
    bool isRunning () const;

    /** @brief Check if a TCP port is opened
     * @return True if open, false otherwise */
    bool isTcpPortOpen () const;

    /** @brief Opens an Ensenso device
     * @param[in] serial The camera serial
     * @return True if successful, false otherwise */
    bool openDevice (std::string serial);


        /** @brief Opens an Ensenso mono device
     * @param[in] serial The camera serial
     * @return True if successful, false otherwise */
    bool openMonoDevice (std::string serial);

    /** @brief Open TCP port to enable access via the