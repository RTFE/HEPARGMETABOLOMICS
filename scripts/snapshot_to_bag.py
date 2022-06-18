#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from genpy.rostime import Time
from rosbag.bag import Bag


def callback_rgb(data, time, bag):
    print 'received PointCloud2!'
    data.header.stamp = time + rospy.Duration.f