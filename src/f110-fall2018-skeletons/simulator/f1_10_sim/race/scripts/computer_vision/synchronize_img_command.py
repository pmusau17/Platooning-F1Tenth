#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped


from race.msg import drive_param


class MessageSynchronizer:
    ''' Gathers messages with vehicle information that have similar time stamps

        /camera/zed/rgb/image_rect_color/compressed: 18 hz
        /camera/zed/rgb/image_rect_color: 18 hz
        /racecar/drive_parameters: 40 hz
    '''
    def __init__(self):
        self.image_rect_color=Subscriber('/racecar/camera/zed/rgb/image_rect_color',Image)
        self.image_rect_color_cp=Subscriber('/racecar/camera/zed/rgb/image_rect_color/compressed',CompressedImage)
        self.drive_parameters=Subscriber('/racecar/drive_parameters',drive_param)
        self.ackermann_stamped=Subscriber('vesc/ackermann_cmd_mux/input/teleop',AckermannDriveStamped)

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.image_rect_color,self.image_rect_color_cp,self.ackermann_stamped], queue_size = 20, slop = 0.049)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

    def master_callback(self,image,image_cp,drive_parameters): #drive_param):
        print(image,image_cp,drive_parameters)

    def print_cb(self,msg):
        print(msg)


if __name__=='__main__':
    rospy.init_node('image_command_sync')
    
    #initialize the message filter
    mf=MessageSynchronizer()
    #rospy.Subscriber(il.image_topic,drive_param,mf.print_cb)
    #spin so that we can receive messages
    rospy.spin()    