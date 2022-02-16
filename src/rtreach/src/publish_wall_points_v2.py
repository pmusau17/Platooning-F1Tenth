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
        self.pub = rospy.Publisher('wallpoints',reach_tube,queue_size=1)
        self.intervals = []
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
        print(self.wallpoints)
        #self.intervals = 
        
    def odom_callback(self,odom_msg):
        # position 
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        pt = np.asarray([[x,y]]).reshape((-1,2))
        dists = np.linalg.norm(self.wallpoints - pt,axis=-1)
        relevant_indexes=np.where(dists<2.0)[0]
        relevant_points = self.wallpoints[relevant_indexes]
        interval_list = []
        markerArray = MarkerArray()
        for i in range(relevant_points.shape[0]):

            # Interval Portion 
            point = relevant_points[i]
            intv = interval()
            intv.x_min = float(point[0])
            intv.x_max = float(point[0])
            intv.y_min = float(point[1])
            intv.y_max = float(point[1])
            interval_list.append(intv)
        msg = reach_tube()
        msg.obstacle_list = interval_list
        msg.header.stamp = rospy.Time.now()
        msg.count = len(interval_list)
        self.pub.publish(msg)

if __name__=="__main__":
    rospy.init_node("publish_wall_markers")
    args = rospy.myargv()[1:]
    obstacle_path_name=args[0]
    gm = publish_wallpoints(obstacle_file=obstacle_path_name)
    #gm.execute()
    rospy.spin()