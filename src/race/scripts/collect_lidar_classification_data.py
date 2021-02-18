#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
import os
import rospkg
import numpy as np

# import sys so we can use packages outside of this folder in
# either python 2 or python 3, I know it's janky, chill
import sys
import os

class MessageSynchronizer:
    ''' Gathers messages with vehicle information that have similar time stamps
    
        /racecar/scan': 40 hz
        /vesc/ackermann_cmd_mux/input/teleop: 40 hz
    '''
    def __init__(self,racecar_name,vesc_name,data_path):
        self.drive_topic = vesc_name+'/ackermann_cmd_mux/input/teleop'
        self.lidar_topic = racecar_name+'/scan'
        print(self.drive_topic,self.lidar_topic)

        # subscripe to the ackermann commands and lidar topic
        self.ackermann_stamped=Subscriber(self.drive_topic,AckermannDriveStamped)
        self.lidar_sub=Subscriber(self.lidar_topic,LaserScan)

        # get the path to where the data will be stored
        r = rospkg.RosPack()
        self.save_path_root=os.path.sep.join([r.get_path('race'),data_path])
        self.count=0
        self.save_count=0
        self.indices = [180, 300, 360, 420, 540, 660, 720, 780, 900]


        self.filename=self.save_path_root+'{}.csv'.format("/lidar_classification_data_1ms_walker")
        self.file = open(self.filename, 'a')
        print(self.filename)
        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.ackermann_stamped,self.lidar_sub], queue_size = 20, slop = 0.049)
        
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

    #callback for the synchronized messages
    #Note: a negative value means turning to the right, a postive value means turning to the left
    def master_callback(self, ackermann_msg,lidar_msg):
        #convert the steering command to a string to I can store it with the image name
        #for efficient data storage
        command=ackermann_msg.drive.steering_angle
        #replace the period with ~ so it's a valid filename
        #command=command.replace('.','~')
        
        #print(command)
        limited_ranges=np.asarray(lidar_msg.ranges)
        indices=np.where(limited_ranges>=10.0)[0]
        limited_ranges[indices]=10.0
        #print(limited_ranges[self.indices])
        row = list(limited_ranges[self.indices])+[command]
        row_str = '{}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n'.format(*row)
        self.file.write(row_str)
        
    def shutdown(self):
        self.file.close()
        print('Goodbye')
        

if __name__=='__main__':
    rospy.init_node('image_command_sync')

    args = rospy.myargv()[1:]
    
    # get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    
    # get the name of the vesc for the car
    vesc_name=args[1]

    # path where to store the dataset
    data_path = args[2]
    
    # initialize the message filter
    mf=MessageSynchronizer(racecar_name,vesc_name,data_path)
    
    # spin so that we can receive messages
    while not rospy.is_shutdown():
        pass 
    mf.shutdown()