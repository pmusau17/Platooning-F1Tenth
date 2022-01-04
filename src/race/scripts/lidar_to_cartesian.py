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


from tf2_ros import *
import tf2_geometry_msgs



class LidarToCartestian:

    def __init__(self,racecar_name):

        # setting up the tf2 transforms 
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer)

        self.racecar_name = racecar_name
        self.publisher = rospy.Publisher('lidar_to_cartesian', MarkerArray, queue_size="1")
        self.sub = rospy.Subscriber(racecar_name+'/scan', LaserScan, self.lidar_callback)
        
        

    
    def lidar_callback(self,data):

        ranges=data.ranges
        #convert the range to a numpy array so that we can process the data
        limited_ranges=np.asarray(ranges)
        indices=np.where(limited_ranges>=10.0)[0]
        limited_ranges[indices]=(data.range_max)-0.1
        markerArray = MarkerArray()
        # loop through the lidar points (smh I don't like this)
        for i in range(len(limited_ranges)):

            point_base = tf2_geometry_msgs.PointStamped()
            point_base.header.frame_id = self.racecar_name+'/laser'

            angle=(i-540)/4.0
            rad=(angle*math.pi)/180

            point_base.point.x = limited_ranges[i]*math.cos(rad)
            point_base.point.y = limited_ranges[i]*math.sin(rad)
            point_base.point.z = 0.0
            point_base.header.stamp = rospy.get_rostime()

            # Now transform the point into the /odom frame...
            try:
                # The available transforms may be running behind the time
                # stamp on the data.  tf will raise an extrapolation exception
                # if we ask it to transform a point with a "future" time stamp.
                # This call waits (up to 1.0 second) for the necessary
                # transform to become available.

                point_odom = self.tfBuffer.transform(point_base, self.racecar_name+'/odom',
                                                        rospy.Duration(0.1))

                print(i,point_odom.point.x,point_odom.point.y)
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.id = i
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.pose.orientation.w = 1.0
                
                marker.pose.position.x = point_odom.point.x
                marker.pose.position.y = point_odom.point.y
                marker.pose.position.z = point_odom.point.z
                markerArray.markers.append(marker)
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                print("Transform Failed",e)

        #print(markerArray)
        self.publisher.publish(markerArray)


if __name__ == "__main__":
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    racecar_name=args[0]

    rospy.init_node('lidar_to_cartesian', anonymous=True)
    ltc = LidarToCartestian(racecar_name)
    rospy.spin()