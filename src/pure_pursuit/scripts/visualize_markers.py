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
#get the path for this paackage
package_path=rospack.get_path('pure_pursuit')

filename=os.path.sep.join([package_path,'waypoints',waypoint_file])
with open(filename) as f:
	path_points = [tuple(line) for line in csv.reader(f)]

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size="1")
rospy.init_node('register')

# Visualize every other marker to save on memory and speed
while not rospy.is_shutdown():
        markerArray = MarkerArray()
        for i in range(len(path_points)):
                if i % 2 == 0:
                        point = path_points[i]
		        x = float(point[0])
		        y = float(point[1])
		        marker = Marker()
		        marker.header.frame_id = "/map"
		        marker.type = marker.SPHERE
		        marker.action = marker.ADD
		        marker.scale.x = 0.1
		        marker.scale.y = 0.1
		        marker.scale.z = 0.1
		        marker.color.a = 1.0
		        marker.color.r = 1.0
		        marker.color.g = 1.0
		        marker.color.b = 0.0
		        marker.pose.orientation.w = 1.0
		        marker.pose.position.x = x
		        marker.pose.position.y = y
		        marker.pose.position.z = 0

		        markerArray.markers.append(marker)
        
	# Renumber the marker IDs
	id = 0
	for m in markerArray.markers:
		m.id = id
		id += 1

	# Publish the MarkerArray
	publisher.publish(markerArray)

	rospy.sleep(5.0)
