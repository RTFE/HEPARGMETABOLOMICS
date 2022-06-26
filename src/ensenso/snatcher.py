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
    @type  proj