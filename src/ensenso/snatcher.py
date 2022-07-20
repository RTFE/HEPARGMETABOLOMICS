#!/usr/bin/env python
import os
import copy
import rospy
import criros
import numpy as np
import dynamic_reconfigure.client
# OpenCV and PCL
import cv2
from cv_bridge import CvBridge, CvBridgeError
try:
  import pcl
except ImportError:
  raise Exception('pcl python biddings not found: https://github.com/strawlab/python-pcl')
# Messages
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class EnsensoDriverReconfigure():
  def __init__(self, namespace='/', timeout=30):
    ns = criros.utils.solve_namespace(namespace)
    self.dynclient = dynamic_reconfigure.client.Client(ns+'ensenso_driver', timeout=timeout, config_callback=self.cb_dynresponse)

  def cb_dynresponse(self, config):
    """
    TODO: Check that the configuration succeeded.
    """
    pass

  def get_configuration(self, timeout=None):
    return self.dynclient.get_configuration(timeout=timeout)

  def update_configuration(self, config):
    return self.dynclient.update_configuration(config)

  def enable_lights(self, projector=False, frontlight=False):
    """
    Switches on/off the projector and/or the frontlight
    @type  projector: bool
    @param projector: Switch on/off the projector
    @type  frontlight: bool
    @param frontlight: Switch on/off the frontlight
    """
    self.dynclient.update_configuration({'Projector':projector, 'FrontLight':frontlight})

  def enable_streaming(self, cloud=False, images=False):
    """
    Enable/disable the streaming of the point cloud and/or the images
    @type  cloud: bool
    @param cloud: Enable/disable the streaming of the point cloud
    @type  images: bool
    @param images: Enable/disable the streaming of the images
    """
    self.dynclient.update_configuration({'Cloud':cloud, 'Images':images})


class Snatcher(object):
  """
  Class to 'snatch' images and cloud from the ensenso camera. It subscribes to the camera
  topics and connects to the dynamic reconfiguration server to start/stop streaming and
  to switch on/off the projector and frontlight.
  """
  def __init__(self, use_cv_types=True):
    """
    Snatcher constructor. It subscribes to the following topics:
      - Raw images: C{left/image_raw} and C{left/image_raw} of type C{sensor_msgs/Image}
      - Rectified images: C{left/image_rect} and C{left/image_rect} of type C{sensor_msgs/Image}
      - Point cloud: C{depth/points} of type C{sensor_msgs/PointCloud2}
    @type  use_cv_types: bool
    @param use_cv_types: If true will convert image messages to valid OpenCV type.
    @note: If your B{topics are different} use ros remapping.
    """
    self.initialized = False
    # Config stuff
    self.use_cv_types = use_cv_types
    self.bridge = CvBridge()
    self.cv_type = 'mono8'
    # Setup publishers and subscribers
    self.reset_snapshots()
    self.info_left = None
    self.info_right = None
    topics = []
    topics.append(['left/camera_info', CameraInfo, self.cb_info_left])
    topics.append(['right/camera_info', CameraInfo, self.cb_info_right])
    topics.append(['left/image_raw', Image, self.cb_raw_left])
    topics.append(['right/image_raw', Image, self.cb_raw_right])
    topics.append(['left/image_rect', Image, self.cb_rect_left])
    topics.append(['right/image_rect', Image, self.cb_rect_right])
    topics.append(['depth/points', PointCloud2, self.cb_point_cloud])
    self.subscribers = dict()
    for name,ttype,callback in topics:
      self.subscribers[name] = rospy.Subscriber(name, ttype, callback,
                                                                  queue_size=1)
    # Camera configuration client
    self.dynclient = EnsensoDriverReconfigure(namespace=rospy.get_namespace())
    rospy.on_shutdown(self.cb_shutdown)
    self.initialized = True

  def cb_dynresponse(self, config):
    """
    TODO: Check that the configuration succeeded.
    """
    pass

  def cb_info_left(self, msg):
    self.info_left = copy.deepcopy(msg)
    self.subscribers['left/camera_info'].unregister()

  def cb_info_right(self, msg):
    self.info_right = copy.deepcopy(msg)
    self.subscribers['right/camera_info'].unregister()

  def cb_point_cloud(self, msg):
    """
    Callback executed every time a point cloud is received
    @type  msg: sensor_msgs/PointCloud2
    @param msg: The C{PointCloud2} message.
    """
    self.point_cloud = msg
    self.headers['point_cloud'] = copy.deepcopy(msg.header)

  def cb_raw_left(self, msg):
    """
    Callback executed every time a left raw image is received
    @type  msg: sensor_msgs/Image
    @param msg: The C{Image} message.
    """
    self.headers['raw_left'] = copy.deepcopy(msg.header)
    if self.use_cv_types:
      try:
        self.raw_left = self.bridge.imgmsg_to_cv2(msg, self.cv_type)
      except:
        rospy.logdebug('Failed to process raw_left image')
        self.raw_left = None
    else:
      self.raw_left = msg

  def cb_raw_right(self, msg):
    """
    Callback executed every time a right raw image is received
    @type  msg: sensor_msgs/Image
    @param msg: The C{Image} message.
    """
    self.headers['raw_right'] = copy.deepcopy(msg.header)
    if self.use_cv_types:
      try:
        self.raw_right = self.bridge.imgmsg_to_cv2(msg, self.cv_type)
      except:
        rospy.logdebug('Failed to process raw_right image')
        self.raw_right = None
    else:
      self.raw_right = msg

  def cb_rect_left(self, msg):
    """
    Callback executed every time a left rectified imag