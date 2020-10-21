#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import rospkg
import os
import csv
import pdb


#get the arguments passed from the launch file
args = rospy.myargv()[1:]

# get the path to the file containing the waypoints
waypoint_file=args[0]

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

#get the path for this package
package_path=rospack.get_path('race')

filename=os.path.sep.join([package_path,'maps',waypoint_file])
with open(filename) as f:
    path_points = f.read().split("\n")
    #print(len(path_points))

topic = 'wall_points'
publisher = rospy.Publisher(topic, MarkerArray, queue_size="1")
rospy.init_node('wall_points')

# Visualize every other marker to save on memory and speed

markerArray = MarkerArray()
for i in range(len(path_points)):
    if i % 2 == 0:
        point = path_points[i].split(',')
        try:
            x = float(point[0])
            y = float(point[1])
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            markerArray.markers.append(marker)
        except:
            continue

while not rospy.is_shutdown():
    publisher.publish(markerArray)
