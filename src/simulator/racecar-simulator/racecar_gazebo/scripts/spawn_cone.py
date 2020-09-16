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


class SpawnCones():
    def __init__(self,seed):
        """ Randomly spawn a cone within the car's environment so that I can generate scenarios with cones in random locations 
        seed (int) - seed used for random location of the boxes
        """
        np.random.seed(seed)
        self.model_path = RosPack().get_path('racecar_description')
        # open and read the cone file
        self.cone_model = open(os.path.join(self.model_path,"models","cone","model.sdf")).read()
        self.gazebo_namespace = "/gazebo"

        self.cone_markers = markerArray = MarkerArray()
        self.cones_pub = rospy.Publisher('porto_cones', MarkerArray, queue_size="1")
        self.count =0 
        self.cone_name = "Cone"
        self.robot_namespace  = rospy.get_namespace().replace('/', '')
        self.reference_frame = "/map"



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
        marker.header.frame_id = "/map"
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
        marker.pose.position.z = 0
        self.count+=1
        return cone_pose, marker 


    def publish_markers(self):
         self.cones_pub.publish( self.cone_markers)

    def call_spawn_service(self,x,y):

        cp, marker = self.specify_cone_pose_and_marker(x,y)
        gazebo_interface.spawn_sdf_model_client(self.cone_name + str(self.count),self.cone_model,
                                                    self.robot_namespace,
                                                    cp,
                                                    self.reference_frame,
                                                    self.gazebo_namespace)
        self.cone_markers.markers.append(marker)
        print("done")

    def generate_cones(self):
        spawn_locations = [[2.0,2.0],[4.7,2.7],[11.36,-1.46],[3.0,6.4],[-9.64,2.96]]
        spawn_locations = np.asarray(spawn_locations)
        for i in range(spawn_locations.shape[0]):
            self.call_spawn_service(spawn_locations[i][0],spawn_locations[i][1])




if __name__ == "__main__":
    rospy.init_node("spawn_cones")
    rospy.loginfo("spawn obstacles script started")
    args = rospy.myargv()[1:]
    random_seed=int(args[0])
    sp = SpawnCones(random_seed)
    sp.generate_cones()
    while not rospy.is_shutdown():
        sp.publish_markers()





