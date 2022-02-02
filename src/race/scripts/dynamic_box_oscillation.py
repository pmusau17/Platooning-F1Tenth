#!/usr/bin/env python
import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np 
from geometry_msgs.msg import Twist
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class BoxOscillation:

    def __init__(self,box_name,p1,p2):

        self.set_points = [p2,p1]

        # Publisher for 'drive_parameters' (speed and steering angle)
        self.pub = rospy.Publisher(box_name+'/cmd_vel', Twist, queue_size=1)

        self.sub = rospy.Subscriber(box_name+"/odom", Odometry, self.callback, queue_size=1)
        self.count = 0 

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

        vel_msg = Twist()

        x_error = self.set_points[self.count][0]-x
        y_error = self.set_points[self.count][1]-y

        dist = np.asarray([x_error,y_error]).reshape((1,2))
        dist = np.linalg.norm(dist,axis=-1)
        if(dist<0.15):
            self.count = (self.count+1) %2

        vel_msg.linear.x = np.clip(x_error,-1.0,1.0)
        vel_msg.linear.y = np.clip(y_error,-1.0,1.0)
        self.pub.publish(vel_msg)

if __name__ == "__main__":
    #get the arguments passed from the launch file
    
    rospy.init_node('dynamic_box_oscillation', anonymous=True)
    args = rospy.myargv()[1:]
    box_name=args[0]
    x_init = args[1]
    y_init = args[2]
    x_set = args[3]
    y_set = args[4]
    p1 = [float(x_init),float(y_init)]
    p2 = [float(x_set),float(y_set)]
    BoxOscillation(box_name,p1,p2)
    rospy.spin()



      


