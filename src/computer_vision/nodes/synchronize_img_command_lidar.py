#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage,LaserScan
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
import imutils
from race.msg import drive_param
import os
import rospkg
import numpy as np

# import sys so we can use packages outside of this folder in
# either python 2 or python 3, I know it's janky, chill
import sys
import os
from pathlib import Path 
#insert parent directory into the path
sys.path.insert(0,str(Path(os.path.abspath(__file__)).parent.parent))
from preprocessing.utils import ImageUtils 


class MessageSynchronizer:
    ''' Gathers messages with vehicle information that have similar time stamps

        /camera/zed/rgb/image_rect_color/compressed: 18 hz
        /camera/zed/rgb/image_rect_color: 18 hz
        /vesc/ackermann_cmd_mux/input/teleop: 40 hz
    '''
    def __init__(self,racecar_name,vesc_name,data_path):


        self.image_topic = racecar_name+'/camera/zed/rgb/image_rect_color'
        self.drive_topic = vesc_name+'/ackermann_cmd_mux/input/teleop'
        self.lidar_topic = racecar_name+'/scan'
        print(self.image_topic,self.drive_topic,self.lidar_topic)
        self.image_rect_color=Subscriber(self.image_topic,Image)
        self.ackermann_stamped=Subscriber(self.drive_topic,AckermannDriveStamped)
        self.lidar_sub=Subscriber(self.lidar_topic,LaserScan)
        r = rospkg.RosPack()
        self.util=ImageUtils()
        self.save_path_root=os.path.sep.join([r.get_path('computer_vision'),data_path])
        self.cv_bridge=CvBridge()
        self.count=0
        self.save_count=0

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.image_rect_color,self.ackermann_stamped,self.lidar_sub], queue_size = 20, slop = 0.08)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

    #callback for the synchronized messages
    #Note: a negative value means turning to the right, a postive value means turning to the left
    def master_callback(self,image,ackermann_msg,lidar_msg): #drive_param):
        #convert rosmsg to cv image
        try:
            cv_image=self.cv_bridge.imgmsg_to_cv2(image,"bgr8")
            self.count+=1
        except CvBridgeError as e:
            print(e)

        #convert the steering command to a string to I can store it with the image name
        #for efficient data storage
        command='%.10f' % ackermann_msg.drive.steering_angle
        #replace the period with ~ so it's a valid filename
        command=command.replace('.','~')
        
        #save path 
        save_path=os.path.join(self.save_path_root,self.label_image(ackermann_msg.drive.steering_angle),str(rospy.Time.now())+'~'+command+'.png')

        limited_ranges=np.asarray(lidar_msg.ranges)
        indices=np.where(limited_ranges>=10.0)[0]
        limited_ranges[indices]=10.0
        limited_ranges= limited_ranges[29:1053]
        limited_ranges = limited_ranges.reshape((32,32,1))
        limited_ranges = limited_ranges
        
        if(self.count % 1==0):
            dirPath = os.path.split(save_path)[0]
            if  not 'straight' in dirPath and 'weak_right' not in dirPath and 'weak_left' not in dirPath:
                self.save_image(cv_image,save_path)
                np.save(save_path.replace(".png",".npy"),limited_ranges)
                self.save_count+=1
        self.count+=1
        
    #function that categorizes images into left, weak_left, straight, weak_right, right
    def label_image(self,steering_angle):
        if(steering_angle<-0.261799):
            return "right"
        elif(steering_angle>0.261799):
            return "left"
        elif(steering_angle<-0.0523599 and steering_angle>-0.261799):
            return "weak_right"
        elif(steering_angle>0.0523599 and steering_angle<0.261799):
            return "weak_left"
        else:
            return "straight"

    
    def save_image(self,image,path):
        dirPath = os.path.split(path)[0]
        # if the output directory does not exist, create it
        if not os.path.exists(dirPath):
            os.makedirs(dirPath)
            print('does not exist')
        print(path)
        cv2.imwrite(path,image)

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
    rospy.spin()    