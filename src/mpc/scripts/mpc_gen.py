#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import rospkg 

import do_mpc
from do_mpc.tools.timer import Timer


from template_model import template_model
from template_mpc import template_mpc
from solving import find_constraint

class MPC: 

    # Constructor
    def __init__(self):
        self.lidar = None
        self.drive_publish = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
        self.pose_msg = None
        self.goal_point = None
        # instatntiate subscribers
        rospy.Subscriber('racecar/odom', Odometry, self.pose_callback, queue_size=1)
        rospy.Subscriber('racecar/goal_point', MarkerArray, self.goal_callback, queue_size=1)


    def goal_callback(self,goal_point):
        self.goal_point = goal_point
    def pose_callback(self,pose_msg):
        self.pose_msg = pose_msg
        quaternion = np.array([pose_msg.pose.pose.orientation.x,
                            pose_msg.pose.pose.orientation.y,
                            pose_msg.pose.pose.orientation.z,
                            pose_msg.pose.pose.orientation.w])

        position = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z]

        quaternion_z = pose_msg.pose.pose.orientation.z
        quaternion_w = pose_msg.pose.pose.orientation.w

        head_angle = math.atan2(2 * (quaternion_z * quaternion_w), 1 - 2 * (quaternion_z * quaternion_z))

        self.mpc_drive(position[0], position[1], head_angle)
        

    # Pass relevant information to publisher
    def mpc_drive(self,posx, posy, head_angle):

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        if(self.goal_point):
            point = self.goal_point.markers[0]
            tarx,tary = point.pose.position.x,point.pose.position.y
        else: 
            tarx,tary = -1,-1
        if(tarx==-1 and tary==-1):
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed = 0.3
        else:
            model = template_model()
            mpc = template_mpc(model, tarx, tary)
            x0 = np.array([posx, posy, head_angle]).reshape(-1, 1)
            mpc.x0 = x0
            mpc.set_initial_guess()
            
            u0 = mpc.make_step(x0)
            print(u0)
            drive_msg.drive.steering_angle = float(u0[1])
            drive_msg.drive.speed = float(u0[0])
        self.drive_publish.publish(drive_msg)



if __name__ == '__main__':
    rospy.init_node('mpc_node')
    rospy.sleep(5)
    mpc = MPC()
    rospy.spin()
    
    