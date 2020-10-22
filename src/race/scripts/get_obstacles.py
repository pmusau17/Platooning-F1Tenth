#!/usr/bin/env python
import os
import sys

import rospy
import numpy as np
from tf.transformations import quaternion_from_euler
from rospkg import RosPack
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import GetModelState
import csv
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray






def markerCallback(msg):
    markers = msg.markers
    marker_positions = []
    for marker in markers:
        x  = marker.pose.position.x
        y  = marker.pose.position.y
        point = [x,y]
        marker_positions.append(point)
    print(marker_positions)



if __name__=="__main__":
    rospy.init_node("get_obstacles")
    rospy.Subscriber('obstacle_locations', MarkerArray, markerCallback, queue_size=1)
    rospy.spin()

