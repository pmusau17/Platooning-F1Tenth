#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
from std_msgs.msg import Float32
from race.msg import stamped_ttc

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self,racecar_name):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """

        self.ttc_pub=rospy.Publisher(racecar_name+'/ttc',stamped_ttc,queue_size=10)
        # Initialize subscribers
        self.scan_subscriber=Subscriber(racecar_name+'/scan',LaserScan,queue_size=10)
        self.odom_subscriber=Subscriber(racecar_name+'/odom',Odometry,queue_size=10)

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.scan_subscriber,self.odom_subscriber], queue_size = 100, slop = 0.05)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

        self.THRESHOLD=0.5

    def master_callback(self,scan_msg,odom_msg):
        
        degrees_to_radians= np.pi/180
        linear_velocity = np.asarray([odom_msg.twist.twist.linear.x,odom_msg.twist.twist.linear.y])
        linear_velocity = np.linalg.norm(linear_velocity)
        angles_radian=(((np.arange(0,len(scan_msg.ranges))-539)/4))*degrees_to_radians
        projections=np.cos(angles_radian)*linear_velocity
        projections=np.maximum(projections,0.00000001)
        
        # the velocity we get from odom is in the x direction  
        
        ranges = scan_msg.ranges

        # compute the time to collision
        ttc = ranges / projections
        
        # minimum ttc
        minimum_ttc = min(ttc)

        msg= stamped_ttc()
        msg.header.stamp=rospy.Time.now()
        msg.ttc = minimum_ttc
        self.ttc_pub.publish(msg)

def main():
    rospy.init_node('ttc_node')
    # get the arguments from the command line
    args = rospy.myargv()[1:]
    #get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    sn = Safety(racecar_name)
    rospy.spin()
if __name__ == '__main__':
    main()