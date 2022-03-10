#!/usr/bin/env python

import rospy
from race.msg import velocity_msg
from race.msg import angle_msg
from race.msg import drive_param

#need to subscribe to the steering message and angle message
from message_filters import ApproximateTimeSynchronizer, Subscriber


class DecisionManager:
    def __init__(self,racecar_name):
        self.racecar_name=racecar_name

        #subscribe to the angle and the speed message
        self.angle_sub=Subscriber('/'+self.racecar_name+'/angle_msg',angle_msg,queue_size=20)
        self.velocity_sub=Subscriber('/'+self.racecar_name+'/velocity_msg',velocity_msg,queue_size=20)
        self.pub=rospy.Publisher(self.racecar_name+'/drive_parameters',drive_param,queue_size=20)

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.angle_sub,self.velocity_sub], queue_size = 5)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

    def master_callback(self,angle_msg,velocity_msg):
        msg=drive_param()
        msg.header.stamp=rospy.Time.now()
        msg.angle=angle_msg.steering_angle
        msg.velocity=velocity_msg.velocity
        self.pub.publish(msg)



if __name__=="__main__":
    #get the arguments passed from the launch file 
    args = rospy.myargv()[1:]
    racecar_name=args[0]
    dm = DecisionManager(racecar_name)
    rospy.init_node('decision_manager_'+racecar_name, anonymous=True)
    rospy.spin()

