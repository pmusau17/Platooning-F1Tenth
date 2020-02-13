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
        self.classes=['left','right','straight','weak_left','weak_right']

    #image callback
    def image_callback(self,ros_image):
        #convert the ros_image to an openCV image
        try:
            orig_image=self.cv_bridge.imgmsg_to_cv2(ros_image,"bgr8")
            cv_image=self.util.reshape_image(orig_image,self.height,self.width)
            #print(cv_image.shape)
        except CvBridgeError as e:
            print(e)

        predict_image=np.expand_dims(cv_image, axis=0)
        pred=self.model.predict(predict_image)
        cv2.imshow(self.classes[pred[0].argmax()],predict_image[0])
        print(self.classes[pred[0].argmax()])
        cv2.imshow("Original Image",orig_image)
        cv2.waitKey(3) 


if __name__=='__main__':
    rospy.init_node("classify_node",anonymous=True)
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    #get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    #get the keras model
    model=args[1]
    il=ROS_Classify(racecar_name,model,32,32)
    image_sub=rospy.Subscriber(il.image_topic,Image,il.image_callback)
    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()