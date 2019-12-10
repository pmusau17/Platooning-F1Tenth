#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
from race.msg import drive_param
import numpy as np



def process_lidar_data_test(data):
	#get the computed distances from the lidar
	lidar_distances=data.ranges
	print("Number of distances:",len(lidar_distances))
	window=1
	#from calculation the 180th index denotes perpendicular right direction in lidar scan
	right_dist_perp=lidar_distances[180-window:180+window+1]
	#from calculation the 900th index denotes perpendicular left direction in lidar scan
	left_dist_perp=lidar_distances[900-window:900+window+1]
	#from calculation the forward straight ahead index is 540
	straight_ahead=lidar_distances[540-window:540+window+1]
	#from calculation 45 degrees right of straight ahead is 360
	forty_five_right_from_straight=lidar_distances[360-window:360+window+1]
	#from calculation 45 degrees left of straight ahead is 360
	forty_five_left_from_straight=lidar_distances[720-window:720+window+1]
	print("left distance: ",left_dist_perp)
	print("right distance: ",right_dist_perp)
	print("straight distance: ",straight_ahead)
	print("45L: ",forty_five_left_from_straight)
	print("45R: ",forty_five_right_from_straight)


if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('drive_race_patrick',anonymous = True)
	rospy.Subscriber("scan",LaserScan,process_lidar_data_test)
	rospy.spin()
