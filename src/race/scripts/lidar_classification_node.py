#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
#import the tensorflow package
from race.msg import drive_param
from keras.models import load_model
import os
import numpy as np

# import sys so we can use packages outside of this folder in
# either python 2 or python 3, I know it's janky, chill
import sys
import os

class LidarFNNNode:
    ''' Gathers messages with vehicle information that have similar time stamps
    
        /racecar/scan': 40 hz
        /vesc/ackermann_cmd_mux/input/teleop: 40 hz
    '''
    def __init__(self,racecar_name,model):
        self.lidar_topic = racecar_name+'/scan'
        self.model=load_model(model)

        # subscripe to the ackermann commands and lidar topic
        self.pub=rospy.Publisher(racecar_name+'/drive_parameters', drive_param, queue_size=5)
        self.lidar_sub=rospy.Subscriber(self.lidar_topic,LaserScan,self.master_callback)

        # get the path to where the data will be stored
        self.indices = [180, 300, 360, 420, 540, 660, 720, 780, 900]

    #callback for the synchronized messages
    #Note: a negative value means turning to the right, a postive value means turning to the left
    def master_callback(self,lidar_msg):
        
        limited_ranges=np.asarray(lidar_msg.ranges)
        indices=np.where(limited_ranges>=10.0)[0]
        limited_ranges[indices]=10.0

        inp = limited_ranges[self.indices]
        inp = np.expand_dims(inp, axis=0)

        pred=self.model.predict(inp)
        print(pred[0])

        msg = drive_param()
        msg.header.stamp=rospy.Time.now()
        msg.angle = pred[0]
        msg.velocity = 1.0
        # msg.velocity = 0.5
        self.pub.publish(msg)
        
        

if __name__=='__main__':
    rospy.init_node('image_command_sync')

    args = rospy.myargv()[1:]
    
    # get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    

    # path where to store the dataset
    model_path = args[1]
    
    # initialize the message filter
    mf=LidarFNNNode(racecar_name,model_path)
    
    # spin so that we can receive messages
    while not rospy.is_shutdown():
        pass 