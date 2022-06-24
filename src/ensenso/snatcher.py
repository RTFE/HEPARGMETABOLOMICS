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
  def 