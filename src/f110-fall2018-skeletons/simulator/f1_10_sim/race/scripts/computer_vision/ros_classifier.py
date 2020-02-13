#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import imutils

#import the tensorflow package
from tensorflow.python.keras.models import load_model

#import the preprocessing utils (helps with loading data, preprocessing)
from preprocessing.utils import ImageUtils

class ROS_Classify:

    #define the constructor 
    def __init__(self,racecar_name,model,height,width):
        self.cv_bridge=CvBridge()
        self.image_topic=str(racecar_name)+'/camera/zed/rgb/image_rect_color'
        self.model=load_model(model)
        #this handles the reshaping
        self.util=ImageUtils()
        self.width=width
        self.height=height

    #image callback
    def image_callback(self,ros_image):
        #convert the ros_image to an openCV image
        try:
            cv_image=self.cv_bridge.imgmsg_to_cv2(ros_image,"bgr8")
            cv_image=self.util.reshape_image(cv_image,self.height,self.width)
            print(cv_image.shape)
        except CvBridgeError as e:
            print(e)
        cv2.imshow(self.image_topic,cv_image)
        cv2.waitKey(3)
        #img = cv2.resize(cv_image,(224,224),interpolation=cv2.INTER_AREA)
        cv_image= imutils.resize(cv_image,width=200)
        print(cv_image.shape)
        cv2.imshow('224 x 224 topic',cv_image)
        cv2.waitKey(3)
        


if __name__=='__main__':
    rospy.init_node("classify_node",anonymous=True)
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    #get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    #get the keras model
    model=args[1]
    il=ROS_Classify(racecar_name,model)
    image_sub=rospy.Subscriber(il.image_topic,Image,il.image_callback)
    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()