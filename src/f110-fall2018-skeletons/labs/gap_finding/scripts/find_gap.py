#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import numpy as np

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback that receives LIDAR data on the /scan topic.
# data: the LIDAR data, published as sensor_msgs::LaserScan
def scan_callback(data):
	msg = drive_param()
	msg.velocity = 0.1  # TODO: implement PID for velocity
	msg.angle = 0.0    # TODO: implement PID for steering angle
	pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('gap_finding_node', anonymous=True)
	rospy.Subscriber("scan", LaserScan, scan_callback)
	rospy.spin()

