#! /usr/bin/env python
"""
Author : Varundev Suresh Babu
Version: 0.1
"""

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

steering_publisher = rospy.Publisher("throttle_control", Float64, queue_size = 10)

max_swing = 270 # Hokuyo URG-10LX
max_dist = 6.5
min_dist = 0.5
max_vel = 0.5 # Start with 0.25 if using a VESC-X
min_vel = 0.15

def laser_callback(data):
    dist = data.ranges[max_swing/2]
    if dist > max_dist:
        dist = max_dist
    if dist < min_dist:
        dist = min_dist
    vel = Float64()
    vel.data = min_vel + (dist/(max_dist - min_dist))*(max_vel - min_vel)
    steering_publisher.publish(vel)

if __name__ == '__main__':
    rospy.init_node('throttle_control_node')
    rospy.Subscriber("scan", LaserScan, laser_callback)

