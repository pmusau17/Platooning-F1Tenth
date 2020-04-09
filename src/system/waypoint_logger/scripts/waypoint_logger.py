#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

home = expanduser('~')
file = open(strftime(home+'/Documents/catkin_ws/src/f110-fall2018-skeletons/labs/wall_following/logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.txt', 'w')


def checkdistance(arr1,arr2):
	distarr=abs(arr1-arr2)
	allsmall=(distarr<0.2)
	if False in allsmall:
		return True
	else:
		return False


point=np.array([1000000,100000,10000,1000000])
def save_waypoint(data):
    global point
    quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])

    euler = tf.transformations.euler_from_quaternion(quaternion)
    speed = LA.norm(np.array([data.twist.twist.linear.x, 
                              data.twist.twist.linear.y, 
                              data.twist.twist.linear.z]),2)
    if data.twist.twist.linear.x>0.:
        print data.twist.twist.linear.x
    currentPoint=np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    if (checkdistance(currentPoint,point)):
    	file.write('(%f, %f, %f, %f\n),' % (data.pose.pose.position.x,data.pose.pose.position.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))
    else:
	print("Too Close")
    point=np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
def shutdown():
    file.close()
    print('Goodbye')
 
def listener():
    rospy.init_node('waypoints_logger', anonymous=True)
    rospy.Subscriber('pf/pose/odom', Odometry, save_waypoint)
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving waypoints...')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
