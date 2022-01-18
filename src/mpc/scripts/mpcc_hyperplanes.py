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
import copy 


from template_model import template_model
from template_mpc import template_mpc
#from computing_hyperplanes_final import find_constraints


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
import rospkg 
import csv


class MPCC: 

    # Constructor
    def __init__(self,waypoint_file,obstacle_file):
        self.log_hypers = False
        self.use_map  = False
        self.display_in_rviz = False
        self.use_pure_pursuit = True
        self.increment = 1

        self.tar_x = 0 
        self.tar_y = 0
        self.tar_theta = 0
        self.x_min = -100
        self.x_max = 100
        self.y_min = -100
        self.y_max = 100

        self.a0 = 1
        self.b0 = 0
        self.a1 = -1
        self.b1 = 1

        self.count = 0
        # mpc horizon
        self.horizon = 5
        # set up the model used for the mpc controller
        self.model =  template_model()

        # set up the mpc controller
        self.mpc = template_mpc(self.model, self.horizon, -20, -20, 20, 20)  

        self.iter_time = rospy.Time.now()

        # set up the time varying function for mpc
        self.mpc.set_tvp_fun(self.change_target_position_template)
        self.mpc.setup()

        self.left_points = []
        self.right_points = []

        # pt = [1.9937239510681324, 1.2498857949496691]
        # self.left_points.append(pt)

        self.read_waypoints(waypoint_file,obstacle_file)
        self.vis_pub = rospy.Publisher('hyper_planes', MarkerArray,queue_size=100)
        self.drive_publish = rospy.Publisher('/vesc2/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
        self.vis_pub2 = rospy.Publisher("wallpoint_classification", MarkerArray, queue_size=10)

        # instantiate the subscribers

        self.lidar_sub = Subscriber('racecar2/scan', LaserScan)
        self.odom_sub  = Subscriber('racecar2/odom', Odometry)
        self.reach_sub = Subscriber('racecar/reach_tube', reach_tube)
        self.pp_sub = Subscriber('racecar2/goal_point', MarkerArray)

      
        self.u0 = [0,0]

         
   

        #create the time synchronizer
        self.main_sub = ApproximateTimeSynchronizer([self.lidar_sub,self.odom_sub,self.reach_sub,self.pp_sub], queue_size = 20, slop = 0.019,allow_headerless=True)
        
        #register the callback to the synchronizer
        self.main_sub.registerCallback(self.main_callback)


    # this function is for changing the target position without having to 
    # reframe the mpc problem
    def change_target_position_template(self, _):
        """
        Following the docs of do_mpc, an approach to populate the target position variables with values, at any given \
        point.
        """
        template = self.mpc.get_tvp_template()
        print("Change_Target:",self.tar_x,self.tar_y)
        for k in range(self.horizon + 1):
            template["_tvp", k, "target_x"] = self.tar_x
            template["_tvp", k, "target_y"] = self.tar_y
            template["_tvp", k, "a0"] = self.a0
            template["_tvp", k, "b0"] = self.b0
            template["_tvp", k, "a1"] = self.a1
            template["_tvp", k, "b1"] = self.b1
            template["_tvp",k,"target_theta"] = self.tar_theta
            # template["_tvp", k, "x_min"] = self.x_min
            # template["_tvp", k, "x_max"] = self.x_max
            # template["_tvp", k, "y_min"] = self.y_min
            # template["_tvp", k, "y_max"] = self.y_max

        return template


    # convert lidar scans to cartesian point
    def lidar_to_cart(self,ranges, position_x, position_y, heading_angle, starting_index):

        points = []
        markerArray = MarkerArray()
        for index, lidar_range in enumerate(ranges):
            curr_index = starting_index + index
            if(curr_index>360 and curr_index<720):
                continue
            angle=((starting_index + index)-540)/4.0
            rad=(angle*math.pi)/180
            laser_beam_angle = rad

            rotated_angle = laser_beam_angle + heading_angle

            # the 0.265 is the lidar's offset along the x-axis of the car
            # it's in the xacro file
            x_coordinate = (lidar_range) * math.cos(rotated_angle) + position_x + 0.265*math.cos(heading_angle)
            y_coordinate = (lidar_range) * math.sin(rotated_angle) + position_y + 0.265*math.sin(heading_angle)
            #points.append(CartesianPoint(x_coordinate, y_coordinate))
            
            p3 = [x_coordinate,y_coordinate]
                        
            if(curr_index>540):
                self.left_points.append(p3)
            else:
                self.right_points.append(p3)
        # self.left_points =  [l_xmax,l_ymax,l_xmin,l_ymin]
        # self.right_points = [r_xmin,r_ymin,r_xmax,r_ymax]
        # p1 = [l_xmax,l_ymax]
        # p2 = [l_xmin,l_ymin]
        # p3 = [r_xmin,r_ymin]
        # p4 = [r_xmax,r_ymax]
            
        
        #return p1,p2,p3,p4

        


    # Import waypoints.csv into a list (path_points)
    def read_waypoints(self,waypoint_file,obstacle_file):

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        #get the path for the waypoints
        package_path=rospack.get_path('pure_pursuit')
        filename=os.path.sep.join([package_path,'waypoints',waypoint_file])

        # list of xy pts 
        self.xy_points = self.read_points(filename)

        # eulers for 
        self.eulers  = []
        with open(filename) as f:
            self.eulers = [tuple(line)[2] for line in csv.reader(f)]
        self.eulers = np.asarray(self.eulers).astype('double')
        print(self.eulers)

        # get path to obstacle points
        package_path=rospack.get_path('race')
        filename=os.path.sep.join([package_path,'maps',obstacle_file])

        # list of wall_points
        self.wall_points = self.read_points(filename)

        

    def read_points(self,filename):

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        path_points_x   = np.asarray([float(point[0]) for point in path_points])
        path_points_y   = np.asarray([float(point[1]) for point in path_points])

        # list of xy pts 
        xy_points = np.hstack((path_points_x.reshape((-1,1)),path_points_y.reshape((-1,1)))).astype('double')
        return xy_points


    """
    Helper Functions
    """

    def is_left(self,p1,p2,p3):
        return ((p2[0] - p1[0])*(p3[1] - p1[1]) - (p2[1] - p1[1])*(p3[0] - p1[0])) > 0

    """
    Main Callback 
    """

    # The main callback functions of mpc are called within this callback
    def main_callback(self,lidar_data,pose_msg,hypes,pp_point):
        
        orig_point = copy.deepcopy(pp_point)
        lidar_data = np.asarray(lidar_data.ranges)
        indices = np.where(lidar_data>10)[0]
        lidar_data[indices] = 10
        quaternion = np.array([pose_msg.pose.pose.orientation.x,
                            pose_msg.pose.pose.orientation.y,
                            pose_msg.pose.pose.orientation.z,
                            pose_msg.pose.pose.orientation.w])

        position = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z]

        euler = tf.transformations.euler_from_quaternion(quaternion)

        quaternion_z = pose_msg.pose.pose.orientation.z
        quaternion_w = pose_msg.pose.pose.orientation.w

        # #head_angle = math.atan2(2 * (quaternion_z * quaternion_w), 1 - 2 * (quaternion_z * quaternion_z))
        head_angle = euler[2]

        pos_x,pos_y = position[0],position[1]
        pos_1x, pos1_y = pos_x + math.cos(head_angle) * 1.0, pos_y + math.sin(head_angle)*1.0
        

        curr_pos= np.asarray([pos_x,pos_y]).reshape((1,2))
        dist_arr = np.linalg.norm(self.wall_points-curr_pos,axis=-1)

        ##finding those points which are less than 3m away 
        relevant_points = np.where((dist_arr < 10.0))[0]
        
        if(self.use_map):
            # finding the goal point which is within the goal points 
            pts = self.wall_points[relevant_points]

            if(len(pts)>0):
                p1 = (pos_x,pos_y)
                p2 = (pos_1x,pos1_y)
                for pt in pts:
                    p3 = (pt[0],pt[1])

                    if(self.is_left(p1,p2,p3)):
                        self.left_points.append(p3)
                    else:
                        self.right_points.append(p3)

            if(len(self.left_points)>0 and len(self.right_points)>0):

                self.left_points  = np.asarray(self.left_points).reshape((-1,2))
                self.right_points  = np.asarray(self.right_points).reshape((-1,2))

                

                curr_pos= np.asarray([pos_x,pos_y]).reshape((1,2))
                dist_arr = np.linalg.norm(self.left_points-curr_pos,axis=-1)
                dist_arr2 = np.linalg.norm(self.right_points-curr_pos,axis=-1)

                dist = 3.0 

                left_point = self.left_points[np.argmin(dist_arr)]
                lx, ly = left_point[0] + math.cos(head_angle) * dist, left_point[1] + math.sin(head_angle)*dist
                lx1, ly1 = left_point[0] + math.cos(head_angle) * (-dist), left_point[1] + math.sin(head_angle)*(-dist)
                line1 = [lx1,ly1,lx,ly]

                right_point = self.right_points[np.argmin(dist_arr2)]
                rx1, ry1 = right_point[0] + math.cos(head_angle) * (-dist), right_point[1] + math.sin(head_angle) * (-dist)
                rx, ry = right_point[0] + math.cos(head_angle) * dist, right_point[1] + math.sin(head_angle)*dist
                line2 = [rx1,ry1,rx,ry]
                self.left_points = []
                self.right_points = []
                
                self.visualize_lines([line1,line2])
        else:

            #self.lidar_to_cart(lidar_data[180:901],pos_x,pos_y,head_angle,180)
            self.lidar_to_cart(lidar_data,pos_x,pos_y,head_angle,0)
            self.left_points  = np.asarray(self.left_points).reshape((-1,2))
            self.right_points  = np.asarray(self.right_points).reshape((-1,2))
            curr_pos= np.asarray([pos_x,pos_y]).reshape((1,2))
            dist_arr = np.linalg.norm(self.left_points-curr_pos,axis=-1)
            dist_arr2 = np.linalg.norm(self.right_points-curr_pos,axis=-1)
            dist_arr3 = np.linalg.norm(self.xy_points - curr_pos,axis=-1)

            dist = 3.0 
            

            #lps = self.left_points[max(np.argmin(dist_arr)-20,0):np.argmin(dist_arr)+20]
            #print(lps)
            # lx, ly  = self.left_points[-1]
            # lx1, ly1 = self.left_points[np.argmin(dist_arr)]
           

            center_angle = self.eulers[np.argmin(dist_arr3)]

            

            left_point = self.left_points[np.argmin(dist_arr)]
            #left_point = self.left_points[-1]
            lx, ly = left_point[0] + math.cos(center_angle) * dist, left_point[1] + math.sin(center_angle)*dist
            lx1, ly1 = left_point[0] + math.cos(center_angle) * (-dist), left_point[1] + math.sin(center_angle)*(-dist)
            line1 = [lx,ly,lx1,ly1]
            
            
            right_point = self.right_points[np.argmin(dist_arr2)]
            #right_point = self.right_points[0]
            rx1, ry1 = right_point[0] + math.cos(center_angle) * (-dist), right_point[1] + math.sin(center_angle) * (-dist)
            rx, ry = right_point[0] + math.cos(center_angle) * dist, right_point[1] + math.sin(center_angle)*dist
            line2 = [rx,ry,rx1,ry1]

            p1 = self.xy_points[np.argmin(dist_arr3)]
            #print(center_angle,p1)
            #self.left_points = [p1]
            #self.right_points = []
            #self.visualize_points()
            
            # #right_point = self.right_points[np.argmin(dist_arr2)]
            # #print(np.argmin(dist_arr2))
            # #rps = self.right_points[max(np.argmin(dist_arr2)-20,0):np.argmin(dist_arr2)+20]
            # #print(rps)
            # rx, ry = self.right_points[0]
            # rx1, ry1 = self.right_points[np.argmin(dist_arr2)]
            

            # m  = (ly1 - ly) / (lx1 - lx)
            # b = ly1 - (m*lx1)

            # m1 = (ry1 - ry) / (rx1 - rx)
            # b1 = ry1 - (m1*rx1)

            # x1 = rx1 -1 
            # x2 = rx1 + 1
            # y1 = m*x1 + b
            # y2 = m*x2 + b

            # x01 = lx -1 
            # x02 = lx + 1
            # y01 = m1*x01 + b1
            # y02 = m1*x02 + b1
            
            # line1 = [x1,y1,x2,y2]
            # line2 = [x01,y01,x02,y02]

            # line1 = [rx1,ry1,x2,y2]
            # line2 = [lx,ly,x02,y02]

            #self.visualize_points()
           
            #self.visualize_points()
            self.left_points = []
            self.right_points = []
            
            # closest center point 
            #self.left_points = [p1]
            #self.visualize_points()

            self.visualize_lines([line1,line2])

        #self.lidar_to_cart(lidar_data[240:840], pos_x, pos_y, head_angle, 240)

        # creating inequalities 
        m  = (ly1 - ly) / (lx1 - lx)
        b = ly1 - (m*lx1)

        
        m1 = (ry1 - ry) / (rx1 - rx)
        b1 = ry1 - (m*rx1)

        # print(m,m1)
        # print(0>=((pos_x*m+b)-pos_y), 0>=(pos_y-(pos_x*m1+b1)))
        # print(0>=((pos_x*m1+b1)-pos_y), 0>=(pos_y-(pos_x*m+b)))

        
            
           

        # print("cons1:",0 >= b,0>=((pos_x*m+b)-pos_y))
        # print("cons2:",0 <= b1,0<=((pos_x*m1+b1)-pos_y))
        
        point = orig_point.markers[0]
            
        tarx,tary = point.pose.position.x, point.pose.position.y

        tar_pos   = np.asarray([tarx,tary]).reshape((1,2))
        dist_arr3 = np.linalg.norm(self.xy_points - tar_pos,axis=-1)
        tar_theta = self.eulers[np.argmin(dist_arr3)]


        #print(tarx,tary,tar_theta)
        distance = (self.tar_x - pos_x) ** 2 + (self.tar_y - pos_y) ** 2
            #print("Distance:",distance,"tar_x:",self.tar_x,"tar_y:",self.tar_y,tarx,tary)

            #if(self.count==0 or distance<0.1):
        
        rospy.logwarn("{},{},{},{}".format(point.pose.position.x,point.pose.position.y,self.tar_x,self.tar_y))
        if(self.count==0 or distance<1.9 or (rospy.Time.now()-self.iter_time).to_sec()>15):
            rospy.logwarn("Change Target")
            self.tar_x =  point.pose.position.x
            self.tar_y =  point.pose.position.y
            self.tar_theta  = tar_theta
            self.iter_time = rospy.Time.now()

            if(0>=((pos_x*m+b)-pos_y)):
                self.a0 = m
                self.b0 = b
                self.a1 = m1
                self.b1 = b1
            else:
                self.a0 = m1
                self.b0 = b1
                self.a1 = m
                self.b1 = b


                
        x0 = np.array([pos_x, pos_y, head_angle]).reshape(-1, 1)

        # linear velocity 
        velx = pose_msg.twist.twist.linear.x
        vely = pose_msg.twist.twist.linear.y
        velz = pose_msg.twist.twist.linear.z

        self.visualize_points()


        

        # magnitude of velocity 
        speed = np.asarray([velx,vely])
        speed = np.linalg.norm(speed)

    
        if(self.count==0):
            self.mpc.x0 = x0
            self.mpc.set_initial_guess()

                
        #for i in range(10):
        u0 = self.mpc.make_step(x0)
        self.u0 = u0
            

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = float(u0[1])
        drive_msg.drive.speed = 1.0
        self.drive_publish.publish(drive_msg)

        self.count+=1
    
        rospy.logwarn("count: {}".format(self.count))

        self.left_points = []
        self.right_points = []
        self.left_points = [[self.tar_x,self.tar_y]]
        self.right_points = [[pp_point.markers[0].pose.position.x,pp_point.markers[0].pose.position.y]]
        self.visualize_points()

        # reset left and right points
        
        if(self.count>100):
            self.count = 1
    
    def visualize_points(self,frame='map'):
        # create a marker array
        markerArray = MarkerArray()
        for i in range(len(self.left_points)):
            pt = self.left_points[i]

            x = float(pt[0])
            y = float(pt[1])
            
            marker = Marker()
            marker.id = i 
            marker.header.frame_id = frame
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            markerArray.markers.append(marker)

        for i in range(len(self.right_points)):
            pt = self.right_points[i]

            x = float(pt[0])
            y = float(pt[1])
            
            marker = Marker()
            marker.id = len(self.left_points)+i 
            marker.header.frame_id = frame
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            markerArray.markers.append(marker)
        self.vis_pub2.publish(markerArray)


    def visualize_lines(self, lines):
        
        markerArray = MarkerArray()
        for i in range(2):
            line = lines[i]
            x1,y1,x2,y2 = line

            marker = Marker()
            marker.id = 10000 + i
            marker.header.frame_id = "map"
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
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
    rospy.init_node('mpcc_node')
    waypoint_file = "track_porto_26780.csv"
    obstacle_file = "track_porto_obstacles.txt"
    mpc = MPCC(waypoint_file,obstacle_file)
    rospy.spin()
