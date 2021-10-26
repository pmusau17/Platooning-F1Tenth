#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from race.msg import reach_tube
import sys
import os
#import tf


import math
import numpy as np
import scipy as sp


import pdb
import sys
sys.path.append('../../')
import do_mpc
from do_mpc.tools.timer import Timer


from template_model import template_model
from template_mpc_hyperplanes import template_mpc
from solving import find_constraint

from constants import LEFT_DIVERGENCE_INDEX, RIGHT_DIVERGENCE_INDEX
from constants import FTG_IGNORE_RANGE, SAFETY_RADIUS, POSITION_PREDICTION_TIME
from constants import LIDAR_MINIMUM_ANGLE, LIDAR_ANGLE_INCREMENT, LIDAR_MAX_INDEX
from point import LidarPoint, CartesianPoint
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point


class MPC: 

    # Constructor
    def __init__(self):
        self.lidar = None
        self.hypes = None
        self.drive_publish = rospy.Publisher('/vesc2/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
        self.vis_pub = rospy.Publisher("sanity_pub", MarkerArray, queue_size=10)


        # instatntiate subscribers
        rospy.Subscriber('racecar2/scan', LaserScan, self.odometry_update, queue_size=1)
        rospy.Subscriber('racecar2/odom', Odometry, self.pose_callback, queue_size=1)
        rospy.Subscriber('racecar/reach_tube', reach_tube, self.reach_callback, queue_size=1)


    def find_sequence(self, points, ignore_range):

        current_left_index = 0
        current_right_index = 0
        current_longest_sequence = 0
        final_left_index = 0
        final_right_index = 0
        is_sequence_started = False

        for i, point in enumerate(points):

                # Lengthen the sub-sequence or start a new one if non-ignore number found
            if point.range != ignore_range:
                if is_sequence_started:
                    current_right_index += 1
                else:
                    is_sequence_started = True
                    current_left_index = i
                    current_right_index = i

                # End current sub-sequence and see if it was any bigger than the previous one
            else:
                if is_sequence_started:
                    length = current_right_index - current_left_index
                    if length > current_longest_sequence:
                        final_left_index = current_left_index
                        final_right_index = current_right_index
                        current_longest_sequence = length
                    is_sequence_started = False

        # Finally, make sure that if the last point was also within the sub-sequence, the sub-sequence is also counted
        if is_sequence_started:
            length = current_right_index - current_left_index
            if length > current_longest_sequence:
                final_left_index = current_left_index
                final_right_index = current_right_index

        return points[final_left_index:final_right_index]



    def get_target(self, points):

        if not points:
            return -1,-1

        return points[len(points)//2].cartesian



    # convert lidar scans to cartesian point
    def lidar_to_cart(self,ranges, position_x, position_y, heading_angle, starting_index):

        points = []

        for index, lidar_range in enumerate(ranges):
            laser_beam_angle = ((starting_index + index) * LIDAR_ANGLE_INCREMENT) + LIDAR_MINIMUM_ANGLE
            rotated_angle = laser_beam_angle + heading_angle
            x_coordinate = lidar_range * math.cos(rotated_angle) + position_x
            y_coordinate = lidar_range * math.sin(rotated_angle) + position_y
            points.append(CartesianPoint(x_coordinate, y_coordinate))


        return points

    def adjust_target_position(self, position_x, position_y, heading_angle):
    


        lidar_data = self.lidar

        ranges = lidar_data.ranges[RIGHT_DIVERGENCE_INDEX:(LEFT_DIVERGENCE_INDEX+1)]

        cartesian_points = self.lidar_to_cart(
                ranges=ranges,
                position_x=position_x,
                position_y=position_y,
                heading_angle=heading_angle,
                starting_index=RIGHT_DIVERGENCE_INDEX
            )

            # Build a list of relevant Point instances
        points = list()
        for i in range(LEFT_DIVERGENCE_INDEX - RIGHT_DIVERGENCE_INDEX + 1):
            cartesian_point = cartesian_points[i]
            lidar_index = i + RIGHT_DIVERGENCE_INDEX
            lidar_range = ranges[i]

            if lidar_range <= SAFETY_RADIUS:
                lidar_range = FTG_IGNORE_RANGE

            points.append(LidarPoint(lidar_index, lidar_range, cartesian_point))

        tarx, tary = self.get_target(self.find_sequence(points, FTG_IGNORE_RANGE))

        return tarx, tary
        
            # Pass relevant information to publisher
            
    def overlap(self, mpc_x_min, mpc_x_max, mpc_y_min, mpc_y_max, rectanglexm, rectanglexmx, rectangleym, rectangleymaxx):  # return true if they overalap
    
        if (mpc_x_min  >= rectanglexmx) or (rectanglexm >= mpc_x_max):
            return False
            
        if (mpc_y_max <= rectangleym) or (rectangleymaxx <= mpc_y_min):
            return False
            
        return True  
            
    def mpc_drive(self, posx, posy, head_angle, tarx, tary):

        # prevents nul message errors
        if(self.hypes):

            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            
            rectangle = self.hypes
            
            a, b = find_constraint(posx, posy, rectangle.x_min, rectangle.x_max, rectangle.y_max) 
            print(a, b)         
            
           # for visualization 
            
            #rtreach_interval = [[rectangle.x_min, rectangle.x_max], [rectangle.y_min, rectangle.y_max]]
            #self.visualize_rectangles(mpc_interval,rtreach_interval)
            self.visualize_rectangles()
            
            if(tarx==-1 and tary==-1):
                drive_msg.drive.steering_angle = 0.0
                drive_msg.drive.speed = 0.0
            else:
                model = template_model()
                mpc = template_mpc(model, tarx, tary, a, b)
                x0 = np.array([posx, posy, head_angle]).reshape(-1, 1)
                mpc.x0 = x0
                mpc.set_initial_guess()
                
                u0 = mpc.make_step(x0)

                drive_msg.drive.steering_angle = float(u0[1])
                drive_msg.drive.speed = float(u0[0])
            self.drive_publish.publish(drive_msg)
        
    def find_safes(self, position_x, position_y, heading_angle):
    

        lidar_data = self.lidar

        ranges = lidar_data.ranges[RIGHT_DIVERGENCE_INDEX:(LEFT_DIVERGENCE_INDEX+1)]

        cartesian_points = self.lidar_to_cart(
                ranges=ranges,
                position_x=position_x,
                position_y=position_y,
                heading_angle=heading_angle,
                starting_index=RIGHT_DIVERGENCE_INDEX
            )

            # Build a list of relevant Point instances
        points = list()
        for i in range(LEFT_DIVERGENCE_INDEX - RIGHT_DIVERGENCE_INDEX + 1):
            cartesian_point = cartesian_points[i]
            lidar_index = i + RIGHT_DIVERGENCE_INDEX
            lidar_range = ranges[i]

            if lidar_range <= SAFETY_RADIUS:
                lidar_range = FTG_IGNORE_RANGE

            points.append(LidarPoint(lidar_index, lidar_range, cartesian_point))

        pts = self.find_sequence(points, FTG_IGNORE_RANGE)
        
    
        minx, miny = pts[0].cartesian
        maxx, maxy = pts[len(pts)-1].cartesian
          

        return min(minx, maxx), max(minx, maxx), min(miny, maxy), max(miny, maxy) # minimum x, maximum x, minimum y, maximum y


    def odometry_update(self,data):
        self.lidar = data

    def pose_callback(self,pose_msg):

        quaternion = np.array([pose_msg.pose.pose.orientation.x,
                            pose_msg.pose.pose.orientation.y,
                            pose_msg.pose.pose.orientation.z,
                            pose_msg.pose.pose.orientation.w])

        position = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z]

        #euler = tf.transformations.euler_from_quaternion(quaternion)

        quaternion_z = pose_msg.pose.pose.orientation.z
        quaternion_w = pose_msg.pose.pose.orientation.w

        head_angle = math.atan2(2 * (quaternion_z * quaternion_w), 1 - 2 * (quaternion_z * quaternion_z))

        tarx = self.adjust_target_position(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, head_angle)[0]
        tary = self.adjust_target_position(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, head_angle)[1]
    

        self.mpc_drive(position[0], position[1], head_angle, tarx, tary)
        
    def reach_callback(self, msg):
        if(msg.count>0):
            reach_list = msg.obstacle_list
            last_index = 0 #msg.count-1
            last_rectangle = reach_list[last_index]
            self.hypes = last_rectangle 
            rospy.logwarn("x: [{},{}], y: [{},{}]".format(last_rectangle.x_min,last_rectangle.x_max,last_rectangle.y_min,last_rectangle.y_max))
            

    def visualize_rectangles(self):
        markerArray = MarkerArray()

        #intervals = [mpc_inteval,lidar_interval]
        marker = Marker()
        marker.id = 0
        marker.header.frame_id = "/map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        
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

        self.vis_pub.publish(markerArray)
        

if __name__ == '__main__':
    rospy.init_node('mpc_node')
    mpc = MPC()
    rospy.spin()
    
    
    
    
    
    
