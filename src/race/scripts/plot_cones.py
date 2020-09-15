#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import rospkg
import os
import csv
import pdb
import numpy as np 


cones =  [[[1.935, 2.065],[1.935, 2.065]],[[4.635, 4.765],[2.635, 2.765]],
						[[11.295, 11.425],[-1.525, -1.395]],[[2.935, 3.065],[6.335, 6.465]],[[-9.705, -9.575],[2.895, 3.025]]]




cones = np.asarray(cones)

topic = 'porto_cones'
publisher = rospy.Publisher(topic, MarkerArray, queue_size="1")
rospy.init_node('porto_cone_rviz')

# Visualize every other marker to save on memory and speed
while not rospy.is_shutdown():
    markerArray = MarkerArray()
    for i in range(cones.shape[0]):
        x = (cones[i][0][1] + cones[i][0][0])/2.0
        y = (cones[i][1][1] + cones[i][1][0])/2.0
        xwidth =  (cones[i][0][1] - cones[i][0][0])
        ywidth = (cones[i][1][1] - cones[i][1][0])
        marker = Marker()
        marker.id = i
        marker.header.frame_id = "/map"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = xwidth
        marker.scale.y = ywidth
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.4
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        markerArray.markers.append(marker)
	# Publish the MarkerArray
	publisher.publish(markerArray)
	rospy.sleep(0.3)
