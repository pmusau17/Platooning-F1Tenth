#!/usr/bin/env python

import rospy
from race.msg import drive_param
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

class pure_pursuit:

    def __init__(self,racecar_name,waypoint_file):

        # initialize class fields 
        self.racecar_name = racecar_name
        self.waypoint_file = waypoint_file

        # pure pursuit parameters
        self.LOOKAHEAD_DISTANCE = 1.5#1.70 # meters
        self.VELOCITY = 3.2 # m/s
        self.goal = 0
        self.read_waypoints()
       
        # Publisher for 'drive_parameters' (speed and steering angle)
        self.pub = rospy.Publisher(racecar_name+'/drive_parameters', drive_param, queue_size=1)
        # Publisher for the goal point
        self.goal_pub = rospy.Publisher(racecar_name+'/goal_point', MarkerArray, queue_size="1")
        self.considered_pub= rospy.Publisher(racecar_name+'/considered_points', MarkerArray, queue_size="1")
        # Subscriber to vehicle position 
        rospy.Subscriber(racecar_name+"/odom", Odometry, self.callback, queue_size=1)

    # Import waypoints.csv into a list (path_points)
    def read_waypoints(self):

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        #get the path for this paackage
        package_path=rospack.get_path('a_stars_pure_pursuit')
        filename=os.path.sep.join([package_path,'waypoints',waypoint_file])

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points_x   = np.asarray([float(point[0]) for point in path_points])
        self.path_points_y   = np.asarray([float(point[1]) for point in path_points])
        self.path_points_w   = np.asarray([float(point[2]) for point in path_points])

        self.xy_points = np.hstack((self.path_points_x.reshape((-1,1)),self.path_points_y.reshape((-1,1))))
   
    def visualize_goal_point(self,pt,r=1.0,g=0.0,b=1.0):
        # create a marker array
        markerArray = MarkerArray()
        x = float(pt[0])
        y = float(pt[1])
		
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        markerArray.markers.append(marker)
        self.goal_pub.publish(markerArray)

    def visualize_considered_points(self,pts,r=0.0,g=0.0,b=1.0):

        # create a marker array
        markerArray = MarkerArray()
        
        idx = np.random.randint(0,len(pts))
        pt = pts[idx]

        x = float(pt[0])
        y = float(pt[1])            
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        markerArray.markers.append(marker)

        self.considered_pub.publish(markerArray)

    

    # Input data is PoseStamped message from topic racecar_name/odom.
    # Runs pure pursuit and publishes velocity and steering angle.
    def callback(self,data):

        qx=data.pose.pose.orientation.x
        qy=data.pose.pose.orientation.y
        qz=data.pose.pose.orientation.z
        qw=data.pose.pose.orientation.w

        quaternion = (qx,qy,qz,qw)
        euler = euler_from_quaternion(quaternion)
        yaw   = euler[2] 

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        ## finding the distance of each way point from the current position 
        curr_pos=np.asarray([x,y]).reshape((1,2))
        dist_arr = np.linalg.norm(self.xy_points-curr_pos,axis=-1)

        ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
        #goal_arr = np.where((self.dist_arr < self.LOOKAHEAD_DISTANCE+0.3)&(self.dist_arr > self.LOOKAHEAD_DISTANCE-0.3))[0]
        goal_arr = np.where(dist_arr < self.LOOKAHEAD_DISTANCE)[0]
        
        # finding the goal point which is within the goal points 
        pts = self.xy_points[goal_arr]

        self.visualize_considered_points(pts)

        

        for idx in range(len(pts)): 
            v1 = pts[idx] - curr_pos
            #since the euler was specified in the order x,y,z the angle is wrt to x axis
            v2 = [np.cos(yaw), np.sin(yaw)]

            temp_angle = self.find_angle(v1,v2)
            #print(temp_angle)
            if abs(temp_angle) < np.pi/2:
                self.goal = idx
                self.visualize_goal_point(pts[self.goal])

    
        #self.visualize_considered_points(pts)
    
        # # ##finding the goal point which is the last in the set of points less than the lookahead distance
        # # ##if the closest points array could not be formed, then the point which is closest to the current position is the goal. 
        # for idx in goal_arr:
        #     #line from the point position to the car position
        #     v1 = [self.path_points_x[idx]-x , self.path_points_y[idx]-y]

        #     #since the euler was specified in the order x,y,z the angle is wrt to x axis
        #     v2 = [np.cos(yaw), np.sin(yaw)]
        #     #find the angle between these two vectors NOTE:These are in world coordinates
        #     temp_angle = self.find_angle(v1,v2)
        #     if abs(temp_angle) < np.pi/2:
        #         self.goal = idx
        #         #print("broke")
        #         break

        # ##finding the distance of the goal point from the vehicle coordinatesr

        # L = self.dist_arr[self.goal]

        # ##Transforming the goal point into the vehicle coordinate frame (This come straight from the paper)

        # gvcx = self.path_points_x[self.goal] - x
        # gvcy = self.path_points_y[self.goal] - y 
        # goal_x_veh_coord = gvcx*np.cos(yaw) + gvcy*np.sin(yaw)
        # goal_y_veh_coord = gvcy*np.cos(yaw) - gvcx*np.sin(yaw)

        # # math: find the curvature and the angle 
        # #alpha = self.path_points_w[self.goal] - (yaw)
        # if temp_angle>np.pi/2:
        #     temp_angle= yaw

        # #print(temp_angle,self.path_points_w[self.goal])
        # alpha = (temp_angle-yaw)
        # k = 2 * math.sin(alpha)/L
        # angle_i = math.atan(k*0.4)

        # angle = angle_i*2
        # angle = np.clip(angle, -0.6108652353, 0.6108652353) # 0.6108652353 radians = 35 degrees because car can only turn 35 degrees max
        
        # pt = [float(self.path_points_x[self.goal]),self.path_points_y[self.goal]]
        # self.visualize_goal_point(pt)

      
        #self.set_speed(angle)
        #self.const_speed(angle)

	    #publish the goal in the vehicle coordinates. 
	    #goalPoint = Point(float(goal_x_veh_coord),float(goal_y_veh_coord),float(angle))
        #pt = [float(goal_x_veh_coord),float(goal_y_veh_coord)]
        #pt = [float(self.path_points_x[self.goal]),self.path_points_y[self.goal]]
        #self.visualize_goal_point(pt)

   
    # USE THIS FUNCTION IF CHANGEABLE SPEED IS NEEDED
    def set_speed(self,angle):

        msg = drive_param()
        
        if (abs(angle)>0.2018):

            self.LOOKAHEAD_DISTANCE = 1.2
            msg.angle = angle

            if msg.velocity - 1.5 >= 0.5:
                msg.velocity -= 0.5#0.7

        else:
            self.LOOKAHEAD_DISTANCE = 1.2
            msg.angle = angle

            if self.VELOCITY - msg.velocity > 0.2:
                msg.velocity += 0.2
        print(angle,msg.velocity)
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)


    # USE THIS FUNCTION IF CONSTANT SPEED IS NEEDED
    def const_speed(self,angle):
        msg = drive_param()
        msg.header.stamp = rospy.Time.now()
        msg.angle = angle
        msg.velocity = 0.5
        self.pub.publish(msg)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)


if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    # get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    # get the path to the file containing the waypoints
    waypoint_file=args[1]
    C = pure_pursuit(racecar_name,waypoint_file)  
    r = rospy.Rate(40)

    while not rospy.is_shutdown():
        r.sleep()
