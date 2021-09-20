#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import rospy
import math

import os
import csv
import pdb
import numpy as np

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size=98)

rospy.init_node('view_node')

while not rospy.is_shutdown():
    markerArray = MarkerArray()

    marker = Marker()
    marker.id = 0
    marker.header.frame_id = "/map"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.7
    marker.color.b = 0.8

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 0.0

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    marker.points = []
    first_line_point = Point()
    first_line_point.x = 5.5
    first_line_point.y = 2.7
    first_line_point.z = 0.0
    marker.points.append(first_line_point)

    # second point
    second_line_point = Point()
    second_line_point.x = 0.0
    second_line_point.y = -0.3
    second_line_point.z = 0.0
    marker.points.append(second_line_point)



    markerArray.markers.append(marker)

    publisher.publish(markerArray)

    rospy.sleep(0.3)
