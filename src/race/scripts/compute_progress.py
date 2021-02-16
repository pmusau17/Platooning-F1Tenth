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

"""
Class that computes the progress of the F1Tenth around the track. Takes a lot of inspiration from the 
Pure Pursuit Controller.
"""




class ComputeProgress:
    def __init__(self,racecar_name,waypoint_file,log_file):

        # initialize class fields 
        self.racecar_name = racecar_name
        self.waypoint_file = waypoint_file
        self.read_waypoints()

        # Odometry Subscriber
        rospy.Subscriber(racecar_name+"/odom", Odometry, self.callback, queue_size=1)

        # Visualize for debugging
        self.goal_pub = rospy.Publisher(racecar_name+'/closest_point', MarkerArray, queue_size="1")

        # Visualize for debugging
        self.start_pub = rospy.Publisher(racecar_name+'/start_point', MarkerArray, queue_size="1")

        self.initialize_start_point = False
        # self start point
        self.start_point = []

        self.lap_count = 0 
        self.total_elapsed =rospy.Time.now()
        self.start_time = rospy.Time.now()

        # for logging purposes
        rospack = rospkg.RosPack()
        self.progress_file = os.path.join(rospack.get_path('race'),"logs",log_file)
        rospy.logwarn(self.progress_file)

    # Import waypoints.csv into a list (path_points)
    def read_waypoints(self):

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        #get the path for this paackage
        package_path=rospack.get_path('pure_pursuit')
        filename=os.path.sep.join([package_path,'waypoints',self.waypoint_file])

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points_x   = np.asarray([float(point[0]) for point in path_points])
        self.path_points_y   = np.asarray([float(point[1]) for point in path_points])

        # list of xy pts 
        self.xy_points = np.hstack((self.path_points_x.reshape((-1,1)),self.path_points_y.reshape((-1,1)))).astype('double')

        

    # Visualize the goal point    
    def visualize_point(self,pts,publisher,frame='/map',r=1.0,g=0.0,b=1.0):
        # create a marker array
        markerArray = MarkerArray()

        x = float(pts[0])
        y = float(pts[1])
		
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

        idx = dist_arr.argmin() 
        point = np.asarray(self.xy_points[idx,:])

        # Initialize the first point as the start point
        if(not self.initialize_start_point):
            self.start_point = point
            self.start_index = idx
            self.initialize_start_point = True

        # Visualize both points as the start point 
        self.visualize_point(point,self.goal_pub)

        # Visualize both points as the start point 
        self.visualize_point(self.start_point,self.start_pub,r=0.0,g=0.0)

        # current point to starting point
        norm = np.linalg.norm(self.start_point - point) 
        self.progress = idx/float(self.xy_points.shape[0])

        # prevent from counting laps more than once
        td = rospy.Time.now() - self.start_time
        if(self.progress>0.97 and norm < 0.15 and td>rospy.Duration(5)):
            self.lap_count +=1
            self.start_time = rospy.Time.now()

        #print(self.progress,norm,self.lap_count)

    # log the lap count to a file
    def shutdown(self):
        elapsedTime = (rospy.Time.now()-self.total_elapsed).to_sec()
        print("total_laps_completed:",self.lap_count+self.progress,'total_time_taken:',elapsedTime)
        if(os.path.exists(self.progress_file)):
            fi = open(self.progress_file, "a")
            fi.write("{}, {}\n".format(self.lap_count+self.progress,elapsedTime))
            fi.close()
        else: 
            fi = open(self.progress_file, "w")
            fi.write("{}, {}\n".format(self.lap_count+self.progress,elapsedTime))
            fi.close()
        print('Goodbye')

if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    # get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    # get the path to the file containing the waypoints
    waypoint_file=args[1]
    # log file 
    log_file = args[2]
    C = ComputeProgress(racecar_name,waypoint_file,log_file)  

    while not rospy.is_shutdown():
        pass 
    C.shutdown()