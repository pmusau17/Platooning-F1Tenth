#!/usr/bin/env python


"""This script creates an odometric frame so that amcl can use it"""
import rospy 
import tf
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from std_msgs.msg import Header
import tf2_ros

br=tf.TransformBroadcaster()
br2=tf2_ros.TransformBroadcaster()

#get the arguments passed from the launch file 
args = rospy.myargv()[1:]
racecar_name=args[0]

def vesc_odom_callback(data):
    global br
    global racecar_name

    currentTime=data.header.stamp
    rospy.loginfo("currentTime: "+str(currentTime))
    frameID=data.header.frame_id
    rospy.loginfo("Frame ID: "+frameID)
    childFrame=data.child_frame_id
    rospy.loginfo("Child Frame ID: "+childFrame)
    pose=data.pose.pose
    rospy.loginfo("pose: "+str(pose))
    twist=data.twist.twist
    rospy.loginfo("twist: "+str(twist))

    odom_trans=TransformStamped()
    odom_trans.header.stamp=currentTime
    odom_trans.header.frame_id=racecar_name+"/odom"
    odom_trans.child_frame_id=racecar_name+"/base_link"

    #Odometry is the distance of something relative to a point. In our case it is the distance between the base link
    #and the fixed frame odom. Here we attach the odom frame to the map so the transform between odom and base_link 
    #is the same transform as map to base_link
    #This information is given by gazebo so I publish the position and orientation by subscribing to the /vesc/odom topio
    #the transforms are given by position and orientation. To understand this in more detail look at gazeboOdometry.py
    odom_trans.transform.translation.x=pose.position.x
    odom_trans.transform.translation.y=pose.position.y
    odom_trans.transform.translation.z=pose.position.z
    odom_trans.transform.rotation=pose.orientation#Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
    br.sendTransformMessage(odom_trans)

    #Here I want to publish the transform between the odom frame and the map since we fix the odom frame to the map 
    #frame all the transforms are 0
    pose2=data.pose.pose.position
    #("pose"+str(pose2))
    pose2.x=0
    pose2.y=0
    pose2.z=0
    print("pose: "+str(pose2))
    tf1 = TransformStamped(
            header=Header(
                frame_id="map",
                stamp=currentTime
            ),
            child_frame_id=racecar_name+"/odom",
            transform=Transform(
                translation=pose2,
                rotation=Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
            )
        )
    br2.sendTransform(tf1)






def vesc_odom_listener():
    rospy.init_node("message_to_tf_"+racecar_name,anonymous=True)
    rospy.Subscriber("/"+racecar_name+"/odom",Odometry,vesc_odom_callback)
    rospy.spin()

if __name__=="__main__":
    while not rospy.is_shutdown():
        vesc_odom_listener()
