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
      - Raw images: C{left/image_raw} and C{left/image_raw} of 