#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import imutils


class ImageListener:

    #define the constructor 
    def __init__(self,racecar_name):
        self.cv_bridge=CvBridge()

        self.image_topic=str(racecar_name)+'/camera/zed/rgb/image_rect_color'

    #image callback
    def image_callback(self,ros_image):
        #convert the ros_image to an openCV image
        try:
            print(type(ros_image)==Image)
            cv_image=self.cv_bridge.imgmsg_to_cv2(ros_image,"bgr8")
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
    rospy.init_node("image_listener",anonymous=True)
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    racecar_name=args[0]
    il=ImageListener(racecar_name)
    image_sub=rospy.Subscriber(il.image_topic,Image,il.image_callback)
    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()
