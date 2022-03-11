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
from rtreach.msg import reach_tube
from rtreach.msg import interval


class SpawnCones():
    def __init__(self,seed,free_space_path,obstacle_count):
        """ Randomly spawn a cone within the car's environment so that I can generate scenarios with cones in random locations 
        seed (int) - seed used for random location of the boxes
        """
        np.random.seed(seed)
        self.model_path = RosPack().get_path('racecar_description')
        self.freespace_path = RosPack().get_path('race')
        # open and read the cone file
        self.cone_file = open(os.path.join(self.model_path,"models","cone","model.sdf"))
        self.cone_model = self.cone_file.read()
        self.free_space_file = open(os.path.join(self.freespace_path,"maps",free_space_path))
        self.free_space= self.free_space_file.read().split("\n")
        self.gazebo_namespace = "/gazebo"

        self.cone_markers = markerArray = MarkerArray()
        self.cones_pub = rospy.Publisher('obstacle_locations', MarkerArray, queue_size=10)
        self.obs_pub = rospy.Publisher('obstacle_tubes',reach_tube,queue_size=10)
        self.count =0 
        self.cone_name = "Cone"
        self.robot_namespace  = rospy.get_namespace().replace('/', '')
        self.reference_frame = "/map"
        self.num_obsatcles = obstacle_count

        self.intervals = []



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
        marker.header.stamp = rospy.Time.now()
        marker.lifetime = rospy.Duration(10000000)
        marker.header.frame_id = "map"
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
        markers = self.cone_markers
        for m in markers.markers:
            m.header.stamp = rospy.Time.now()
        self.cones_pub.publish( self.cone_markers)

        msg = reach_tube()
        msg.obstacle_list = self.intervals
        msg.header.stamp = rospy.Time.now()
        msg.count = len(self.intervals)
        self.obs_pub.publish(msg)

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
        #spawn_locations = [[2.0,2.0],[4.7,2.7],[11.36,-1.46],[3.0,6.4],[-9.64,2.96]]
        num_free = len(self.free_space)-1
        randindexes= np.arange(num_free)
        np.random.shuffle(randindexes)
        randindexes = list(randindexes)
        for i in range(self.num_obsatcles):
            ind = randindexes.pop(0)
            point = self.free_space[ind].split(',')
            try:
                x,y = float(point[0]),float(point[1])
                intv = interval()
                intv.x_min = x-(0.13)/2
                intv.x_max = x+(0.13)/2
                intv.y_min = y-(0.13)/2
                intv.y_max = y+(0.13)/2
                self.intervals.append(intv)
            except:
                print(point)

            self.call_spawn_service(x,y,randindexes[i])

        # clean up nice
        self.free_space_file.close()
        self.cone_file.close()

    def set_pub(self,pub):
        self.cones_pub = pub




if __name__ == "__main__":
    rospy.init_node("spawn_cones")
    rospy.loginfo("spawn obstacles script started")
    args = rospy.myargv()[1:]
    random_seed=int(args[0])
    rospy.logwarn("random_seed: "+str(random_seed))
    free_space_path=args[1]
    obstacle_count =int(args[2])
    sp = SpawnCones(random_seed,free_space_path,obstacle_count)
    sp.generate_cones()

    r = rospy.Rate(40)
    while not rospy.is_shutdown():
        sp.publish_markers()
        r.sleep()





