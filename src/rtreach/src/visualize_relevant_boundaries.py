#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from rtreach.msg import reach_tube
from rtreach.msg import interval
import numpy as np
import rospkg 
from nav_msgs.msg import Odometry

class publish_wallpoints:
    def __init__(self,obstacle_file='porto',racecar_name='racecar'):
        # use the rospack object to get paths
        rospack = rospkg.RosPack()
        package_path=rospack.get_path('rtreach')
        filename=package_path+'/obstacles/{}_obstacles.txt'.format(obstacle_file)
        f= open(filename, "r")
        points= f.read().split('\n')
        f.close()
        self.points = points
        self.marker_pub = rospy.Publisher('relevant_boundaries', MarkerArray, queue_size="1")
        self.wallpoints = []
        self.load_wallpoints()
        self.sub = rospy.Subscriber(racecar_name+"/odom", Odometry,self.odom_callback,queue_size=10)

    def load_wallpoints(self):
        #interval_list = []
        for i in range(len(self.points)):
            point=self.points[i].split(',')
            if(len(point)<2):
                continue
            self.wallpoints.append([float(point[0]),float(point[1])])
        self.wallpoints = np.asarray(self.wallpoints).reshape((-1,2))
        
    def odom_callback(self,odom_msg):
        # position 
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        pt = np.asarray([[x,y]]).reshape((-1,2))
        dists = np.linalg.norm(self.wallpoints - pt,axis=-1)
        relevant_indexes=np.where(dists<2.0)[0]
        relevant_points = self.wallpoints[relevant_indexes]
        markerArray = MarkerArray()
        for i in range(relevant_points.shape[0]):

            # Interval Portion 
            point = relevant_points[i]
            # Markers 
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "/map"
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
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0
            markerArray.markers.append(marker)
            marker.lifetime =rospy.Duration(0.1)
        self.marker_pub.publish(markerArray)





if __name__=="__main__":
    rospy.init_node("publish_wall_markers")
    args = rospy.myargv()[1:]
    obstacle_path_name=args[0]
    gm = publish_wallpoints(obstacle_file=obstacle_path_name)
    #gm.execute()
    rospy.spin()