#!/usr/bin/env python
import rospy
from rtreach.msg import reach_tube
from rtreach.msg import interval
import numpy as np
import rospkg 

class publish_wallpoints:
    def __init__(self,obstacle_file='porto'):
        # use the rospack object to get paths
        rospack = rospkg.RosPack()
        package_path=rospack.get_path('rtreach')
        filename=package_path+'/obstacles/{}_obstacles.txt'.format(obstacle_file)
        f= open(filename, "r")
        points= f.read().split('\n')
        f.close()
        self.points = points
        self.pub = rospy.Publisher('wallpoints',reach_tube,queue_size=20)
        self.intervals = []
        self.load_wallpoints()

    def load_wallpoints(self):
        interval_list = []
        for i in range(len(self.points)):
            point=self.points[i].split(',')
            if(len(point)<2):
                continue
            intv = interval()
            intv.x_min = float(point[0])
            intv.x_max = float(point[0])
            intv.y_min = float(point[1])
            intv.y_max = float(point[1])
            interval_list.append(intv)
        self.intervals = interval_list

    def execute(self):
        rate = rospy.Rate(80)
        while not rospy.is_shutdown():
            msg = reach_tube()
            msg.obstacle_list = self.intervals
            msg.header.stamp = rospy.Time.now()
            msg.count = len(self.intervals)
            self.pub.publish(msg)
            rate.sleep()



if __name__=="__main__":
    rospy.init_node("publish_wall_markers")
    args = rospy.myargv()[1:]
    obstacle_path_name=args[0]
    gm = publish_wallpoints(obstacle_file=obstacle_path_name)
    gm.execute()
    rospy.spin()