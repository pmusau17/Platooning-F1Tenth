#!/usr/bin/env python
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import rospy
import math
import rospkg
import os
#need to subscribe to the steering message and angle message
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np 

class ProcessDynamicObstacles:
    def __init__(self,racecar_name,wallpoints):
        self.racecar_name = racecar_name
        self.wallpoints = wallpoints
        self.pub = rospy.Publisher(self.racecar_name+"/processed_obstacles", MarkerArray, queue_size="1")

        #subscribe to the viz and the odom message
        self.marker_sub=Subscriber('viz',MarkerArray)
        self.odom_sub=Subscriber(self.racecar_name+'/odom',Odometry)
        self.sub = ApproximateTimeSynchronizer([self.marker_sub,self.odom_sub], queue_size = 20, slop = 0.019,allow_headerless=True)
        self.path = os.path.join(rospkg.RosPack().get_path('race'),'maps',self.wallpoints)
        self.points = self.load_wallpoints()
        self.sub.registerCallback(self.master_callback)
        

    def calculate_intervals(self,center,width=1,height=1):
        '''
            Addition by Patrick for RealTime Reachability
        '''
        x_int = [center[0]-width/2, center[0]+width/2]
        y_int = [center[1]-height/2, center[1]+height/2]
        return x_int,y_int 


    def load_wallpoints(self):
        with open(self.path) as f:
            path_points = f.read().split("\n")
            del path_points[-1]
        path_points = [[float(dx.split(',')[0]),float(dx.split(',')[1])] for dx in path_points]
        path_points = np.asarray(path_points).reshape((-1,2))
        return path_points


    def check_safety(self,obs,point):

        wall_point = [[point[0],point[0]],[point[1],point[1]]]
        l1 = [obs[0][0],obs[1][1]]
        r1 = [obs[0][1],obs[1][0]]

        l2 = [wall_point[0][0],wall_point[1][1]]
        r2 = [wall_point[0][1],wall_point[1][0]]
        if (l1[0] >= r2[0] or l2[0] >= r1[0]):
            return True 
        if (l1[1] <= r2[1] or l2[1] <= r1[1]): 
            return True

	    return False

    def check_intersection_walls(self,xint,yint):
        for pp in self.points:
            add = self.check_safety([xint,yint],pp)
            if(not add):
                break 
        return add 



    def master_callback(self,marker_array_msg,odom_msg):
        # position 
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        odom_point = np.asarray([x,y]).reshape((1,2))

        # Convert Quaternion to rpy
        rpy = euler_from_quaternion([odom_msg.pose.pose.orientation.x,
                                     odom_msg.pose.pose.orientation.y,
                                     odom_msg.pose.pose.orientation.z,
                                     odom_msg.pose.pose.orientation.w])

        # markers
        markerArray = MarkerArray()
        markers = marker_array_msg.markers
        for marker in markers:
            mx = marker.pose.position.x
            my = marker.pose.position.y
            
            marker_point = np.asarray([mx,my]).reshape((1,2))
            diff = np.linalg.norm(odom_point - marker_point)

            # Ignore obstacles that are more than 5.0 meters away, 
            # Change this if needed
            if(diff<5.0):
                marker.id = marker.id+1
                marker.lifetime = rospy.Duration(0.05)
                xi,yi = self.calculate_intervals((mx,my),width=marker.scale.x,height=marker.scale.y)
                if(self.check_intersection_walls(xi,yi)):
                    markerArray.markers.append(marker)
                

        self.pub.publish(markerArray)




if __name__=="__main__":
    #get the arguments passed from the launch file 
    args = rospy.myargv()[1:]
    racecar_name=args[0]
    wallpoints=args[1]
    rospy.init_node('decision_manager_'+racecar_name, anonymous=True)
    dm =  ProcessDynamicObstacles(racecar_name,wallpoints)
    rospy.spin()








