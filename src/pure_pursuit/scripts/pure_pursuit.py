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
        
        # Distance from the 
        self.distance_from_rear_wheel_to_front_wheel = 0.5

        self.VELOCITY = 3.2 # m/s
        self.goal = 0
        self.read_waypoints()
       
        # Publisher for 'drive_parameters' (speed and steering angle)
        self.pub = rospy.Publisher(racecar_name+'/drive_parameters', drive_param, queue_size=1)
        # Publisher for the goal point
        self.goal_pub = rospy.Publisher(racecar_name+'/goal_point', MarkerArray, queue_size="1")
        self.considered_pub= rospy.Publisher(racecar_name+'/considered_points', MarkerArray, queue_size="1")
        self.point_in_car_frame= rospy.Publisher(racecar_name+'/goal_point_car_frame', MarkerArray, queue_size="1")
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

        # list of xy pts 
        self.xy_points = np.hstack((self.path_points_x.reshape((-1,1)),self.path_points_y.reshape((-1,1)))).astype('double')
   
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
        yaw   = np.double(euler[2])

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        ## finding the distance of each way point from the current position 
        curr_pos= np.asarray([x,y]).reshape((1,2))
        dist_arr = np.linalg.norm(self.xy_points-curr_pos,axis=-1)

        ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
        goal_arr = np.where((dist_arr > self.LOOKAHEAD_DISTANCE) & (dist_arr<self.LOOKAHEAD_DISTANCE+0.3))[0]
        
        # finding the goal point which is within the goal points 
        pts = self.xy_points[goal_arr]

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

        vector = np.asarray([xgv,ygv])
        self.visualize_point([vector],self.point_in_car_frame,frame='racecar/chassis',r=0.0,g=1.0,b=0.0)
        
        # calculate the steering angle
        angle = math.atan2(ygv,xgv)
        self.const_speed(angle)
   
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
        cosang = np.dot(v1, v2).astype('double')
        sinang = la.norm(np.cross(v1, v2)).astype('double')
        return np.arctan2(sinang, cosang).astype('double')


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
