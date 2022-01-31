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
from porto_zigzag import points


class SpawnCones():
    def __init__(self):
        """ Randomly spawn a cone within the car's environment so that I can generate scenarios with cones in random locations 
        seed (int) - seed used for random location of the boxes
        """
        self.model_path = RosPack().get_path('racecar_description')
        self.freespace_path = RosPack().get_path('race')
        # open and read the cone file
        self.cone_file = open(os.path.join(self.model_path,"models","cone","model.sdf"))
        self.cone_model = self.cone_file.read()
        self.gazebo_namespace = "/gazebo"

        self.cone_markers = markerArray = MarkerArray()
        self.cones_pub = rospy.Publisher('obstacle_locations', MarkerArray, queue_size=10)
        self.cone_name = "Cone"
        self.robot_namespace  = rospy.get_namespace().replace('/', '')
        self.reference_frame = "/map"
        self.count=0


    def specify_cone_pose_and_marker(self,x,y):

        cone_pose = Pose()
        cone_pose.position.x = x
        cone_pose.position.y = y
        cone_pose.position.z = 0
        
        cone_orient = quaternion_from_euler(0, 0, 0)
        cone_pose.orientation.x = cone_orient[0]
        cone_pose.orientation.y = cone_orient[1]
        cone_pose.orientation.z = cone_orient[2]
        cone_pose.orientation.w = cone_orient[3]


        marker = Marker()
        marker.id = self.count
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.13
        marker.scale.y = 0.13
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.4
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0
        marker.lifetime = rospy.Duration(1)
        self.count+=1
        return cone_pose, marker 


    def publish_markers(self):
        markers = self.cone_markers
        for m in markers.markers:
            m.header.stamp = rospy.Time.now()
        self.cones_pub.publish( self.cone_markers)

    def calculate_intervals(self,center,width=0.13,height=0.13):

        """
        Sanity Check for intervals representing obstacles
        """

        x_int = [center[0]-width/2, center[0]+width/2]
        y_int = [center[1]-height/2, center[1]+height/2]

    def call_spawn_service(self,x,y,index):
        cp, marker = self.specify_cone_pose_and_marker(x,y)
        gazebo_interface.spawn_sdf_model_client(self.cone_name + str(index),self.cone_model,
                                                    self.robot_namespace,
                                                    cp,
                                                    self.reference_frame,
                                                    self.gazebo_namespace)
        self.cone_markers.markers.append(marker)
        print("done")

    def generate_cones(self):
        for i in range(len(points)):
            point = points[i]
            x,y = float(point[0]),float(point[1])
            self.call_spawn_service(x,y,i)
            self.count+=1

        # clean up nice
        self.cone_file.close()




if __name__ == "__main__":
    rospy.init_node("spawn_cones")
    rospy.loginfo("spawn obstacles script started")
    sp = SpawnCones()
    sp.generate_cones()
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
        sp.publish_markers()
        r.sleep()





