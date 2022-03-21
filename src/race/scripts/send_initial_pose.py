#!/usr/bin/env python
import rospy
from race.msg import velocity_msg
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray

class SendInitialPose:

    #default is to publish speed commands of 1 m/s at 40 hz 
    def __init__(self,racecar_name,rate=10.0,speed=1.0):

        self.odometry_sub=rospy.Subscriber(racecar_name+"/odom", Odometry, self.callback, queue_size=1)
        self.init_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.particle_sub = rospy.Subscriber("/pf/viz/particles",PoseArray,self.quit,queue_size=1)
        self.rate=rate
        self.speed=speed

    def callback(self,msg):

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.pose.pose.position = msg.pose.pose.position
        pose_msg.pose.pose.orientation = msg.pose.pose.orientation
        pose_msg.header.stamp=rospy.Time.now()
        self.init_pub.publish(pose_msg)        

    def quit(self,msg):
        rospy.signal_shutdown("Done")  

if __name__=="__main__":
    #get the arguments passed from the launch file 
    args = rospy.myargv()[1:]   
    racecar_name=args[0]
    rospy.init_node('send_initial_pose'+racecar_name,anonymous=True)
    sp=SendInitialPose(racecar_name)
    rospy.spin()
    