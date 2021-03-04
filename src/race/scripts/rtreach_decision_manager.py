#!/usr/bin/env python

import rospy
from race.msg import velocity_msg
from race.msg import angle_msg
from race.msg import drive_param
from std_msgs.msg import Float32

#need to subscribe to the steering message and angle message
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped


class DecisionManager:
    def __init__(self,racecar_name,vesc_name,pass_anyway=False):
        self.racecar_name=racecar_name
        self.pass_anyway = pass_anyway

        #subscribe to the angle and the speed message
        self.angle_sub=Subscriber('/'+self.racecar_name+'/angle_msg',angle_msg)
        self.reach_result=Subscriber('reachability_result',Float32)
        self.velocity_sub=Subscriber('/'+self.racecar_name+'/velocity_msg',velocity_msg)
        self.safety_sub=Subscriber('/'+self.racecar_name+'/safety',AckermannDriveStamped)
        self.pub=rospy.Publisher(vesc_name+'/ackermann_cmd_mux/input/teleop',AckermannDriveStamped,queue_size=20)

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.angle_sub,self.velocity_sub,self.reach_result, self.safety_sub], queue_size = 20, slop = 0.019,allow_headerless=True)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

    def master_callback(self,angle_msg,velocity_msg,reach_result,safety_sub):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = racecar_name+"/base_link"
        
        #print(velocity_msg.velocity,safety_sub.drive.speed)
        msg.drive.acceleration = 1
        msg.drive.jerk = 1
        msg.drive.steering_angle_velocity = 1
        if(reach_result.data or self.pass_anyway):
            msg.drive.speed = velocity_msg.velocity
            msg.drive.steering_angle = angle_msg.steering_angle
        else:
            msg.drive.speed = safety_sub.drive.speed
            msg.drive.steering_angle = safety_sub.drive.steering_angle

        
        self.pub.publish(msg)



if __name__=="__main__":
    #get the arguments passed from the launch file 
    args = rospy.myargv()[1:]
    racecar_name=args[0]
    vesc_name=args[1]
    pass_anyway = False
    if(args[2:]):
        pass_anyway = True
    dm = DecisionManager(racecar_name,vesc_name,pass_anyway=pass_anyway)
    rospy.init_node('decision_manager_'+racecar_name, anonymous=True)
    rospy.spin()