#!/usr/bin/env python

import rospy
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import rospkg 
from message_filters import ApproximateTimeSynchronizer, Subscriber

#This class will follow the lead car in the same fashion as the pure_pursuit algorithm fingers crossed
class follow_lead_pure_pursuit:
    def __init__(self,lead_car,ego_car):

       
        self.goal = 0

        #self.read_waypoints()
        self.msg = drive_param()
        self.msg.velocity = 1.5#1.5

        # Publisher for 'drive_parameters' (speed and steering angle)
        self.pub = rospy.Publisher(ego_car+'/drive_parameters', drive_param, queue_size=1)
        


        #subscribe to the first car's position and the ego position and you need to synchronize the messages to calculate the distance
        #between them 
        self.lead_car_position=Subscriber(lead_car+"_position_gazebo", PoseStamped)
        self.ego_position=Subscriber(ego_car+"_position_gazebo", PoseStamped)
        self.lidar_ego=rospy.Subscriber(ego_car+"/scan",LaserScan,self.scan_callback)
        self.scan_msg=None
        
        #safety clearance
        self.turn_clearance = 0.25

        #position_window_size
        self.window_size=100
        self.window_index=0


        #position_window
        self.position_window=np.zeros([self.window_size,3])

        #create an array that will store the distances 
        self.dist_arr=np.zeros(self.window_size)

        #tuning parameter for the pure_pursuit
        self.LOOKAHEAD_DISTANCE = 0.70#1.70 # meters

        #platoon_distance
        self.platoon_distance=1.0

        #create a message synchronizer 
        self.sub = ApproximateTimeSynchronizer([self.lead_car_position,self.ego_position],queue_size=20,slop=0.050)
        self.proper_index=None
        #register a callback for the synchronized subscriber
        self.sub.registerCallback(self.sync_callback)
    
    #subscriber that synchronizes the vehicle distances
    def  sync_callback(self,lead,ego):
        #print("lead:",lead)

        #we don't want a massive amount of points from which to choose a goal point so this allows us to rewrite points as we move on
        proper_index=self.window_index%self.window_size
        
        #store this index so that we can get this point if there's no better option
        self.proper_index=proper_index

        
        #we also need to store the orientation for pure pursuit so perform this calculation
        lead_quaternion=lead.pose.orientation
        lead_quat=(lead_quaternion.x,lead_quaternion.y,lead_quaternion.z,lead_quaternion.w)
        lead_yaw=euler_from_quaternion(lead_quat)[2]
        #record the x,y, yaw values into the position window
        self.position_window[proper_index]=np.asarray([lead.pose.position.x,lead.pose.position.y,lead_yaw])
        self.window_index+=1

        distance=self.compute_distance(lead.pose.position,ego.pose.position)
        self.pure_pursuit_following(lead.pose,ego.pose,distance)

    def scan_callback(self,data):
        self.scan_msg=data

    #compute the euclidean distance between them
    def compute_distance(self,pos1,pos2):

        position1=np.asarray([pos1.x,pos1.y])
        position2=np.asarray([pos2.x,pos2.y])

        norm=np.sum(np.sqrt(abs(position1-position2))**2)
        #print(position1,position2,norm)
        return norm

    # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)
    

    def pure_pursuit_following(self,lead_position,ego_position,distance):
        qx=ego_position.orientation.x
        qy=ego_position.orientation.y
        qz=ego_position.orientation.z
        qw=ego_position.orientation.w

        quaternion = (qx,qy,qz,qw)
        euler = euler_from_quaternion(quaternion)
        yaw   = euler[2] 

        x = ego_position.position.x
        y = ego_position.position.y

        #print("lead:",(lead_position.position.x,lead_position.position.y),"ego:",(x,y))

        #choose the goal point to be the closest point of where the lead car has been
        for i in range(self.window_size):
            self.dist_arr[i] = self.dist((self.position_window[i][0],self.position_window[i][1]),(x,y))



        goal_arr = np.where((self.dist_arr < self.LOOKAHEAD_DISTANCE+0.3)&(self.dist_arr > self.LOOKAHEAD_DISTANCE-0.3))[0]

        ##finding the goal point which is the last in the set of points less than the lookahead distance
        ##if the closest points array could not be formed, then the point which is closest to the current position is the goal. 
        if len(goal_arr)==0:
            goal=self.proper_index
            print("No goal")
        goal=self.proper_index

        for idx in goal_arr:
            #line from the point position to the car position
            v1 = [self.position_window[idx][0]-x , self.position_window[idx][1]-y]
            #since the euler was specified in the order x,y,z the angle is wrt to x axis
            v2 = [np.cos(yaw), np.sin(yaw)]
            #find the angle between these two vectors NOTE:These are in world coordinates
            temp_angle = self.find_angle(v1,v2)
            if abs(temp_angle) < np.pi/2:
                goal = idx
                # print(self.goal)
                break


        #goal_point
        goal_point=self.position_window[goal]
        ##Transforming the goal point into the vehicle coordinate frame (This come straight from the paper)

        gvcx = goal_point[0] - x
        gvcy = goal_point[1] - y 
        goal_x_veh_coord = gvcx*np.cos(yaw) + gvcy*np.sin(yaw)
        goal_y_veh_coord = gvcy*np.cos(yaw) - gvcx*np.sin(yaw)

        print(goal_x_veh_coord, goal_y_veh_coord)

        # math: find the curvature and the angle 


        alpha = goal_point[2] - (yaw)
        k = 2 * math.sin(alpha)/distance
        angle_i = math.atan(k*0.4)

        angle = angle_i*2
        angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
        self.const_speed(angle)


    
    #safey function for turning, if it rounds the corner first this will be used later
    def adjust_turning_for_safety(self,left_distances,right_distances,angle):
        min_left=min(left_distances)
        min_right=min(right_distances)

        if min_left<=self.turn_clearance and angle>0.0:#.261799:
            rospy.logwarn("Too Close Left: "+str(min_left))
            angle=0.0
        elif min_right<=self.turn_clearance and angle<0.0:#-0.261799:
            rospy.logwarn("Too Close Right: "+str(min_right))
            angle=0.0
           
        else:
            angle=angle
        return angle

    def const_speed(self,angle):
        if(self.scan_msg):
            ranges=np.asarray(self.scan_msg.ranges)

            #you have to pre_process the lidar data to remove inf values
            indices=np.where(ranges>=10.0)[0]
            ranges[indices]=10.0
            
            span_thirty=ranges[540-120:540+121]
            
            closest_index=np.argmin(span_thirty)+420
            rospy.logwarn(str(min(span_thirty))+" "+str(closest_index))

            #the lidar sweeps counterclockwise so right is [0:180] and left is [901:]
            behind_car_right=ranges[0:180]
            behind_car_left=ranges[901:]
            angle=self.adjust_turning_for_safety(behind_car_left,behind_car_right,angle)


            #distance_error
            error=min(span_thirty)-self.platoon_distance
            print(angle)

            desired_speed=1.0+error
            desired_speed=np.clip(desired_speed,0.9,1.1)

            self.msg.angle = angle
            self.msg.velocity = desired_speed#self.VELOCITY
            self.pub.publish(self.msg)




if __name__ == '__main__':

    #get the arguments from the launch file it should have the lead car and the follower car
    args = rospy.myargv()[1:]
    lead_car=args[0]
    ego_car=args[1]

    rospy.init_node('follow_lead_'+ego_car)
    obj=follow_lead_pure_pursuit(lead_car,ego_car)
    while not rospy.is_shutdown():
        continue