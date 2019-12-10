#!/usr/bin/env python

import rospy
from race.msg import drive_param
from ackermann_msgs.msg import AckermannDriveStamped

import math


#get the arguments passed from the launch file 
args = rospy.myargv()[1:]
racecar_name=args[0]
vesc_name=args[1]
pub = rospy.Publisher('/'+vesc_name+'/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=5)

def vel_and_angle(data):
	

	msg = AckermannDriveStamped()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = racecar_name+"/base_link"

	msg.drive.speed = data.velocity
	msg.drive.acceleration = 1
	msg.drive.jerk = 1
	msg.drive.steering_angle = data.angle
	msg.drive.steering_angle_velocity = 1
	# print "velocity", data.velocity
	# print "angle", data.angle

	pub.publish(msg)



def listener():
	rospy.init_node('sim_connect_'+racecar_name, anonymous=True)
	rospy.Subscriber(racecar_name+'/drive_parameters', drive_param, vel_and_angle)
	rospy.spin()





if __name__=="__main__":
	listener()
