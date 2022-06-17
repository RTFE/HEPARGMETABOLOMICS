
#!/usr/bin/env python
import os
import rospy
import argparse
import datetime
import numpy as np
# OpenCV and PCL
import cv2
import pcl
try:
  import ros_numpy
except ImportError:
  raise Exception('ros_numpy package not found: https://github.com/eric-wieser/ros_numpy')
from ros_numpy.point_cloud2 import get_xyz_points, pointcloud2_to_array
# Python ensenso snatcher
from ensenso.snatcher import Snatcher

class Snapshotter(Snatcher):
  def __init__(self, snapshots, exposure_time, light=False):
    super(Snapshotter, self).__init__()
    # Config stuff
    np.set_printoptions(precision=5, suppress=True)
    self.snapshots = snapshots
    self.exposure_time = exposure_time
    self.light = light
    # Create folder where we will save the snapshoots
    now = rospy.get_time()
    stamp = datetime.datetime.fromtimestamp(now).strftime('%Y-%m-%d-%H-%M-%S')
    basepath = os.path.expanduser('~/.ros/snapshooter/')
    self.folder = os.path.join(basepath, stamp)
    try:
      os.makedirs(self.folder)
    except OSError, e:
      pass    # The folder already exist
  
  def execute(self):