#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from race.msg import reach_tube
import sys
import os

import time


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
from computing_hyperplanes_final import find_constraints


from constants import LEFT_DIVERGENCE_INDEX, RIGHT_DIVERGENCE_INDEX
from constants import FTG_IGNORE_RANGE, SAFETY_RADIUS, POSITION_PREDICTION_TIME
from constants import LIDAR_MINIMUM_ANGLE, LIDAR_ANGLE_INCREMENT, LIDAR_MAX_INDEX
from point import LidarPoint, CartesianPoint
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
#need to subscribe to the steering message and angle message
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf


class MPC: 

    # Constructor
    def __init__(self):
        self.log_hypers = False

        self.vis_pub = rospy.Publisher('lidar_pts', MarkerArray,queue_size=100)
        self.drive_publish = rospy.Publisher('/vesc2/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
        self.vis_pub = rospy.Publisher("sanity_pub", MarkerArray, queue_size=10)

        # instantiate the subscribers

        self.lidar_sub = Subscriber('racecar2/scan', LaserScan)
        self.odom_sub  = Subscriber('racecar2/odom', Odometry)
        self.reach_sub = Subscriber('racecar/reach_tube', reach_tube)

        #create the time synchronizer
        self.main_sub = ApproximateTimeSynchronizer([self.lidar_sub,self.odom_sub,self.reach_sub], queue_size = 20, slop = 0.019)
        
        #register the callback to the synchronizer
        self.main_sub.registerCallback(self.main_callback)

    """
    Main Callback 
    """
    # The main callback functions of mpc are called within this callback
    def main_callback(self,lidar_data,pose_msg,hypes):

        quaternion = np.array([pose_msg.pose.pose.orientation.x,
                            pose_msg.pose.pose.orientation.y,
                            pose_msg.pose.pose.orientation.z,
                            pose_msg.pose.pose.orientation.w])

        position = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z]

        euler = tf.transformations.euler_from_quaternion(quaternion)

        quaternion_z = pose_msg.pose.pose.orientation.z
        quaternion_w = pose_msg.pose.pose.orientation.w

        #head_angle = math.atan2(2 * (quaternion_z * quaternion_w), 1 - 2 * (quaternion_z * quaternion_z))
        head_angle = euler[2]

        tarx,tary,_,_,_,_ = self.adjust_target_position(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, head_angle,lidar_data)
        
        self.mpc_drive(position[0], position[1], head_angle, tarx, tary,lidar_data,hypes)


    """
        Helper Functions for most MPC code
    """
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

        # choose the middle point of the points, otherwise return that 
        # a target point was not found. Point here is a sequence of lidar ranges
        # that we can use to follow the gap
        if not points:
            return -1,-1

        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now() 
        marker.id = 9999 
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.0)
          
        marker.pose.position.x = points[len(points)//2].cartesian[0]
        marker.pose.position.y = points[len(points)//2].cartesian[1]
        marker.pose.position.z = -0.075
        markerArray.markers.append(marker)
        
        self.vis_pub.publish(markerArray)
        
        return points[len(points)//2].cartesian



    # convert lidar scans to cartesian point
    def lidar_to_cart(self,ranges, position_x, position_y, heading_angle, starting_index):

        points = []
        markerArray = MarkerArray()
        for index, lidar_range in enumerate(ranges):
        
            angle=((starting_index + index)-540)/4.0
            rad=(angle*math.pi)/180
            laser_beam_angle = rad



            rotated_angle = laser_beam_angle + heading_angle

            # the 0.265 is the lidar's offset along the x-axis of the car
            # it's in the xacro file
            x_coordinate = (lidar_range) * math.cos(rotated_angle) + position_x + 0.265*math.cos(heading_angle)
            y_coordinate = (lidar_range) * math.sin(rotated_angle) + position_y + 0.265*math.sin(heading_angle)
            #points.append(CartesianPoint(x_coordinate, y_coordinate))
            points.append([x_coordinate,y_coordinate])

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now() 
            marker.id = index
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
            marker.lifetime = rospy.Duration(0.0)
          
            marker.pose.position.x = x_coordinate
            marker.pose.position.y = y_coordinate
            marker.pose.position.z = -0.075
            markerArray.markers.append(marker)
        self.vis_pub.publish(markerArray)


        return points

    def adjust_target_position(self, position_x, position_y, heading_angle,lidar_data):

        #lidar_data = self.lidar

        # RDI = 350
        # LDI = 730
        # so the ranges we are looking are +- 47.5 degrees, for points we could possibly select
        ranges = lidar_data.ranges[RIGHT_DIVERGENCE_INDEX:(LEFT_DIVERGENCE_INDEX+1)]

        # convert the lidar scans in this range to cartesian coordinates
        cartesian_points = self.lidar_to_cart(
                ranges=ranges,
                position_x=position_x,
                position_y=position_y,
                heading_angle=heading_angle,
                starting_index=RIGHT_DIVERGENCE_INDEX
            )

        # Build a list of relevant Point instances that are farther than the safety radius 
        # which is defined to be in this case 2 meters. Can probably do this while we convert them to cartesian
        # Also might not be necessary to choose the target angle
        points = list()
        for i in range(LEFT_DIVERGENCE_INDEX - RIGHT_DIVERGENCE_INDEX + 1):
            cartesian_point = cartesian_points[i]
            lidar_index = i + RIGHT_DIVERGENCE_INDEX
            lidar_range = ranges[i]

            if lidar_range <= SAFETY_RADIUS:
                lidar_range = FTG_IGNORE_RANGE

            points.append(LidarPoint(lidar_index, lidar_range, cartesian_point))

        pts = self.find_sequence(points, FTG_IGNORE_RANGE)
        tarx, tary = self.get_target(pts)

        minx, miny = pts[0].cartesian
        maxx, maxy = pts[len(pts)-1].cartesian
    
        return tarx, tary, min(minx, maxx), max(minx, maxx), min(miny, maxy), max(miny, maxy)
        
    def overlap(self, mpc_x_min, mpc_x_max, mpc_y_min, mpc_y_max, rectanglexm, rectanglexmx, rectangleym, rectangleymaxx):  # return true if they overalap
    
        if (mpc_x_min  >= rectanglexmx) or (rectanglexm >= mpc_x_max):
            return False
            
        if (mpc_y_max <= rectangleym) or (rectangleymaxx <= mpc_y_min):
            return False
            
        return True 
    
    def mpc_drive(self, posx, posy, head_angle, tarx, tary,lidar_data,hypes):

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
                
        rectangle = hypes # Get hyper-rectangles of the opponent vehicle
                 
        ar_0 = self.lidar_to_cart(lidar_data.ranges[240:840], posx, posy, head_angle, 240)   # Convert LiDaR points to Cartesian Points
        
        hw_l = np.asarray(ar_0[680-240:840-240])           
        hw_l_filtered = np.vstack((hw_l[:,0][np.logical_not(np.isinf(hw_l[:,0]))], hw_l[:,1][np.logical_not(np.isinf(hw_l[:,1]))])).T   

        hw_r =  np.asarray(ar_0[240-240:480-240])               
        hw_r_filtered = np.vstack((hw_r[:,0][np.logical_not(np.isinf(hw_r[:,0]))], hw_r[:,1][np.logical_not(np.isinf(hw_r[:,1]))])).T   
            
        x_min = min(hw_l_filtered[0][0], hw_l_filtered[len(hw_l_filtered)-1][0], hw_r_filtered[0][0], hw_r_filtered[len(hw_r_filtered)-1][0], posx)
        x_max = max(hw_l_filtered[0][0], hw_l_filtered[len(hw_l_filtered)-1][0], hw_r_filtered[0][0], hw_r_filtered[len(hw_r_filtered)-1][0], posx)
        y_min = min(hw_l_filtered[0][1], hw_l_filtered[len(hw_l_filtered)-1][1], hw_r_filtered[0][1], hw_r_filtered[len(hw_r_filtered)-1][1], posy)
        y_max = max(hw_l_filtered[0][1], hw_l_filtered[len(hw_l_filtered)-1][1], hw_r_filtered[0][1], hw_r_filtered[len(hw_r_filtered)-1][1], posy)
        
        #mpc_interval = [[x_min, x_max],[y_min, y_max]]     
        #self.visualize_rectangles(mpc_interval)
        
  #      if (self.overlap(x_min, x_max, y_min, y_max, rectangle.obstacle_list[0].x_min, rectangle.obstacle_list[0].x_max, rectangle.obstacle_list[0].y_min, rectangle.obstacle_list[0].y_max)):
            # update hw_r_filtered or hw_l_filtered
         #   rectangle_to_array = np.asarray([[rectangle.obstacle_list[0].x_min, rectangle.obstacle_list[0].y_min], [rectangle.obstacle_list[0].x_min, rectangle.obstacle_list[0].y_max], [rectangle.obstacle_list[0].x_max, rectangle.obstacle_list[0].y_min], [rectangle.obstacle_list[0].x_max, rectangle.obstacle_list[0].y_max]])
            
        #    hw_r_filtered_updated = np.vstack((rectangle_to_array, hw_r_filtered)) # create a new array with reachset included
       #     t_start = time.time()
      #      a0, b0, a1, b1 = find_constraints(posx, posy, hw_l_filtered, hw_r_filtered_updated, tarx, tary) # compute coupled-hyperplanes  
     #       t_end = time.time()           
            
    #    else: 
   #         t_start = time.time()
  #          a0, b0, a1, b1 = find_constraints(posx, posy, hw_l_filtered, hw_r_filtered, tarx, tary) # compute coupled-hyperplanes  
 #           t_end = time.time()
        

        
        a0, b0, a1, b1 = find_constraints(posx, posy, hw_l_filtered, hw_r_filtered, tarx, tary)
        pos_1x, pos1_y = posx + math.cos(head_angle) * 1.0, posy + math.sin(head_angle)*1.0

        x1 = posx * math.cos(head_angle) * -1.0
        y1 = a0 * x1 + b0
           
        x2 = pos_1x
        y2 = a0 * x2 + b0 

        y3 = a1 * x1 + b1
        y4 = a1 * x2 + b1 
   

        lines = [[x1,y1,x2,y2],[x1,y3,x2,y4]]


        if (a0 * posx + b0 - posy > 0): # New Lines -- 
            flag0 = 1
        else: 
            flag0 = -1
                
        if (a1 * posx + b1 - posy > 0):
            flag1 = 1
        else: 
            flag1 = -1    

        print(flag0, flag1)  
        self.visualize_lines(lines)
        if(tarx==-1 and tary==-1):
            drive_msg.drive.steering_angle = 0.0
            drive_msg.drive.speed = 0.0
        else:
            model = template_model()
            mpc = template_mpc(model, tarx, tary, a0, b0, a1, b1, flag0, flag1)  
            x0 = np.array([posx, posy, head_angle]).reshape(-1, 1)
            mpc.x0 = x0
            mpc.set_initial_guess()
                
            u0 = mpc.make_step(x0)

            drive_msg.drive.steering_angle = float(u0[1])
            drive_msg.drive.speed = float(u0[0])
            self.drive_publish.publish(drive_msg)
            

    def visualize_lines(self, lines):
        
        markerArray = MarkerArray()
        for i in range(2):
            line = lines[i]
            x1,y1,x2,y2 = line

            marker = Marker()
            marker.id = 2000 + i
            marker.header.frame_id = "map"
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.7
            marker.color.b = 0.8
            
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05


            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.points = []
            first_line_point = Point()
            first_line_point.x = x1
            first_line_point.y = y1
            first_line_point.z = 0.0
            marker.points.append(first_line_point)

            # second point
            second_line_point = Point()
            second_line_point.x = x2
            second_line_point.y = y2
            second_line_point.z = 0.0
            marker.points.append(second_line_point)
            
            markerArray.markers.append(marker)

        self.vis_pub.publish(markerArray)
        
    def visualize_rectangles(self,mpc_inteval):
        markerArray = MarkerArray()

        intervals = [mpc_inteval]

        for i in range(1):
            hull = intervals[i]
            print(hull)

            marker = Marker()
            marker.id = i
            marker.header.frame_id = "map"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            
            marker.pose.position.x = (hull[0][1]+hull[0][0])/2.0
            marker.pose.position.y = (hull[1][0]+hull[1][1])/2.0
            marker.pose.position.z = 0.0


            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = (hull[0][1]-hull[0][0])
            marker.scale.y = (hull[1][1]-hull[1][0])
            marker.scale.z = 0.05
            marker.color.a = 1.0
            if(i==0):
                marker.color.r = 0.0
                marker.color.g = 239.0/255.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            markerArray.markers.append(marker)

        self.vis_pub.publish(markerArray)        

if __name__ == '__main__':
    rospy.init_node('mpc_node')
    mpc = MPC()
    rospy.spin()
    
    
    
    
    
    
