#!/usr/bin/env python

import rospy
from race.msg import drive_param
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
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
        self.msg = drive_param()
        self.msg.velocity = 1.0

        # Publisher for 'drive_parameters' (speed and steering angle)
        self.pub = rospy.Publisher(ego_car+'/drive_parameters', drive_param, queue_size=1)
        
        # Debugging Visualization 
        self.goal_pub = rospy.Publisher(ego_car+'/goal_point', MarkerArray, queue_size="1")


        #subscribe to the first car's position and the ego position and you need to synchronize the messages to calculate the distance
        #between them 
        self.lead_car_position=Subscriber(lead_car+"/odom", Odometry)
        self.ego_position=Subscriber(ego_car+"/odom", Odometry)
        self.lidar_ego=rospy.Subscriber(ego_car+"/scan",LaserScan,self.scan_callback)
        self.scan_msg=None
        
        #safety clearance
        self.turn_clearance = 0.25

        #position_window_size
        self.window_size=100
        self.window_index=0

        #position_window
        self.position_window=np.zeros([self.window_size,2])

        #prior error for PD controller
        self.prior_error=0
        #create an array that will store the distances 
        self.dist_arr=np.zeros(self.window_size)

        #tuning parameter for the pure_pursuit
        self.LOOKAHEAD_DISTANCE = 0.70

        #platoon_distance
        self.platoon_distance=1.0

        #create a message synchronizer 
        self.sub = ApproximateTimeSynchronizer([self.lead_car_position,self.ego_position],queue_size=20,slop=0.050)
        self.proper_index=None
        #register a callback for the synchronized subscriber
        self.sub.registerCallback(self.sync_callback)
    
    #subscriber that synchronizes the vehicle distances
    def  sync_callback(self,lead,ego):
        #we don't want a massive amount of points from which to choose a goal point so this allows us to rewrite points as we move on
        proper_index=self.window_index%self.window_size
    
        #store this index so that we can get this point if there's no better option
        self.proper_index=proper_index

        #we also need to store the orientation for pure pursuit so perform this calculation
        lead_quaternion=lead.pose.pose.orientation
        lead_quat=(lead_quaternion.x,lead_quaternion.y,lead_quaternion.z,lead_quaternion.w)
        lead_yaw=euler_from_quaternion(lead_quat)[2]
        #record the x,y, yaw values into the position window
        #self.position_window[proper_index]=np.asarray([lead.pose.position.x,lead.pose.position.y,lead_yaw])
        self.position_window[proper_index]=np.asarray([lead.pose.pose.position.x,lead.pose.pose.position.y])
        self.window_index+=1

        distance=self.compute_distance(lead.pose.pose.position,ego.pose.pose.position)
        self.pure_pursuit_following(lead.pose.pose,ego.pose.pose,distance)

    def scan_callback(self,data):
        self.scan_msg=data

    #compute the euclidean distance between them
    # Technically this isn't correct.
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

    # Visualize the points for debugging 
    def visualize_point(self,pts,publisher,frame='/map',r=1.0,g=0.0,b=1.0):
        # create a marker array
        markerArray = MarkerArray()

        idx = np.random.randint(0,len(pts))
        pt = pts[idx]

        x = float(pt[0])
        y = float(pt[1])
		
        marker = Marker()
        marker.header.frame_id = frame
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        markerArray.markers.append(marker)
        publisher.publish(markerArray)
    

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
        

        ## finding the distance of each way point from the current position 
        curr_pos= np.asarray([x,y]).reshape((1,2))
        dist_arr = np.linalg.norm(self.position_window-curr_pos,axis=-1)

        ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
        goal_arr = np.where((dist_arr > self.LOOKAHEAD_DISTANCE) & (dist_arr<self.LOOKAHEAD_DISTANCE+0.3))[0]

        # finding the goal point which is within the goal points 
        pts = self.position_window[goal_arr]


        # get all points in front of the car, using the orientation 
        # and the angle between the vectors
        pts_infrontofcar=[]
        for idx in range(len(pts)): 
            v1 = pts[idx] - curr_pos
            #since the euler was specified in the order x,y,z the angle is wrt to x axis
            v2 = [np.cos(yaw), np.sin(yaw)]

            angle= self.find_angle(v1,v2)
            if angle < np.pi/2:
                pts_infrontofcar.append(pts[idx])

        pts_infrontofcar =np.asarray(pts_infrontofcar)

        # compute new distances
        dist_arr = np.linalg.norm(pts_infrontofcar-curr_pos,axis=-1)- self.LOOKAHEAD_DISTANCE
        
        # get the point closest to the lookahead distance
        idx = np.argmin(dist_arr)

        # goal point 
        goal_point = pts_infrontofcar[idx]
        self.visualize_point([goal_point],self.goal_pub)

        # transform it into the vehicle coordinates
        v1 = (goal_point - curr_pos)[0].astype('double')
        xgv = (v1[0] * np.cos(yaw)) + (v1[1] * np.sin(yaw))
        ygv = (-v1[0] * np.sin(yaw)) + (v1[1] * np.cos(yaw))

        # calculate the steering angle
        angle = math.atan2(ygv,xgv)
        self.const_speed(angle,distance)


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

    def const_speed(self,angle,distance):
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
            error=distance-self.platoon_distance

            #compute the previous error
            derivative_term=error-self.prior_error

            #PD Gains
            K_P=1.5
            K_D=0.3
            
            #PID computation
            desired_speed=K_P*error+K_D*derivative_term+0.2
            print(desired_speed,(distance,min(span_thirty)))
            self.prior_error=error

            #We don't want negative speeds and we don't want to move faster than 1.5 m/s
            desired_speed=np.clip(desired_speed,0,1.5)

            self.msg.angle = angle
            self.msg.velocity = desired_speed
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