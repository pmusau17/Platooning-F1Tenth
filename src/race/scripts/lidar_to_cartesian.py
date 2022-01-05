#!/usr/bin/env python

import time
import rospy
import copy
import numpy as np
import math

from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
from race.msg import drive_param
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
from race.msg import angle_msg
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String


from tf2_ros import *
import tf2_geometry_msgs



class LidarToCartestian:

    def __init__(self,racecar_name):

        # setting up the tf2 transforms 
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer)
        self.racecar_name = racecar_name
        self.vis_pub = rospy.Publisher('lidar_to_cartesian', MarkerArray,queue_size=100)

        # this was a massive headache, need to let the publisher register with the master before we start using the 
        # publisher in the subscirber callback
        rospy.sleep(0.5)
        self.sub = rospy.Subscriber(racecar_name+'/scan', LaserScan, self.lidar_callback)
        
    
    # I won't use this now but this works too
    def get_center_line(self,limited_ranges):

        # point along center line lidar scan
        point_base_center = tf2_geometry_msgs.PointStamped()
        point_base_center.header.frame_id = self.racecar_name+'/laser'
        point_base_center.point.x = limited_ranges[540]
        point_base_center.point.y = 0
        point_base_center.point.z = 0.0
        point_base_center.header.stamp =rospy.Time.now() - rospy.Duration(0.1)

        # point at vehicle center
        point_base_vehicle = tf2_geometry_msgs.PointStamped()
        point_base_vehicle.header.frame_id = self.racecar_name+'/laser'
        point_base_vehicle.point.x = 0.0
        point_base_vehicle.point.y = 0.0
        point_base_vehicle.point.z = 0.0
        point_base_vehicle.header.stamp =rospy.Time.now() - rospy.Duration(0.1)
        try:
                # The available transforms may be running behind the time
                # stamp on the data.  tf will raise an extrapolation exception
                # if we ask it to transform a point with a "future" time stamp.
                # This call waits (up to 1.0 second) for the necessary
                # transform to become available.
                point_center_odom = self.tfBuffer.transform(point_base_center, self.racecar_name+'/odom')
                point_base_odom = self.tfBuffer.transform(point_base_vehicle, self.racecar_name+'/odom')
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
                print("Transform Failed",e)

        return point_base_odom,point_center_odom



    def is_left(self,p1,p2,p3):
        return ((p2.point.x - p1.point.x)*(p3.point.y - p1.point.y) - (p2.point.y - p1.point.y)*(p3.point.x - p1.point.x)) > 0
    
    def lidar_callback(self,data):

        ranges=data.ranges

        # get the two points needed to create a line to check which points are to the left and the right


        #convert the range to a numpy array so that we can process the data
        limited_ranges=np.asarray(ranges)
        #indices=np.where(limited_ranges>=10.0)[0]
        #limited_ranges[indices]=(data.range_max)-0.1

        #p1,p2 = self.get_center_line(limited_ranges)

        markerArray = MarkerArray()
        # loop through the lidar points (smh I don't like this)
        # len(limited_ranges)
        for i in range(180,901):

            point_base = tf2_geometry_msgs.PointStamped()
            point_base.header.frame_id = self.racecar_name+'/laser'

            angle=(i-540)/4.0
            rad=(angle*math.pi)/180

            point_base.point.x = limited_ranges[i]*math.cos(rad)
            point_base.point.y = limited_ranges[i]*math.sin(rad)
            point_base.point.z = 0.0
            point_base.header.stamp =rospy.Time.now() - rospy.Duration(0.05)

            # Now transform the point into the /odom frame...
            try:
                # The available transforms may be running behind the time
                # stamp on the data.  tf will raise an extrapolation exception
                # if we ask it to transform a point with a "future" time stamp.
                # This call waits (up to 1.0 second) for the necessary
                # transform to become available.

                point_odom = self.tfBuffer.transform(point_base, "map") 

                #print(i,point_odom.point.x,point_odom.point.y)
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now() 
                marker.id = i
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                
                marker.color.a = 1.0
                if(i>540):#self.is_left(p1,p2,point_odom)
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    
                marker.pose.orientation.w = 1.0
                marker.lifetime = rospy.Duration(0.0)
          
                marker.pose.position.x = point_odom.point.x
                marker.pose.position.y = point_odom.point.y
                marker.pose.position.z = -0.075
                markerArray.markers.append(marker)
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                print("Transform Failed",e)
        try:
            self.vis_pub.publish(markerArray)
        except Exception as e:
            print(e)


if __name__ == "__main__":
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    racecar_name=args[0]

    rospy.init_node('lidar_to_cartesian', anonymous=True)
    ltc = LidarToCartestian(racecar_name)
    rospy.spin()