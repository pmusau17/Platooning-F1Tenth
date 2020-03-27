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
import tensorflow.keras.backend as K

#import the preprocessing utils (helps with loading data, preprocessing)
from preprocessing.utils import ImageUtils

#class needed to publish to car input
from race.msg import drive_param
from race.msg import angle_msg


class ROS_Daev:

    #define the constructor 
    def __init__(self,racecar_name,model,height,width,decoupled=False):
        self.cv_bridge=CvBridge()
        self.image_topic=str(racecar_name)+'/camera/zed/rgb/image_rect_color'
        self.model=load_model(model,custom_objects={'customAccuracy': self.customAccuracy})
        #this handles the reshaping
        self.util=ImageUtils()
        self.width=width
        self.height=height
        #this is if we onlt want it to publish angle messages
        self.decoupled=decoupled
        if (not self.decoupled):
            self.pub=rospy.Publisher(racecar_name+'/drive_parameters', drive_param, queue_size=5)
        else:
            self.pub=rospy.Publisher(racecar_name+'/angle_msg',angle_msg)


    #image callback
    def image_callback(self,ros_image):
        #convert the ros_image to an openCV image
        try:
            orig_image=self.cv_bridge.imgmsg_to_cv2(ros_image,"bgr8")/255.0
            cv_image=self.util.reshape_image(orig_image,self.height,self.width)
            #print(cv_image.shape)
        except CvBridgeError as e:
            print(e)

        predict_image=np.expand_dims(cv_image, axis=0)
        pred=self.model.predict(predict_image)[0]*0.6108652353

        #Want to keep things consistent
        if (not self.decoupled):
            msg = drive_param()
            msg.header.stamp=rospy.Time.now()
            msg.angle = pred
            msg.velocity = 1.0
        else:
            msg=angle_msg()
            msg.header.stamp=rospy.Time.now()
            msg.steering_angle=pred
        self.pub.publish(msg)
        
        cv2.imshow("Image fed to network",predict_image[0])
        print(str(pred))
        cv2.imshow("Original Image",orig_image)
        cv2.waitKey(3) 
    
    #define a custom metric for DAEV, accuracy doesn't cut it
    def customAccuracy(self,y_true, y_pred):
        diff = K.abs(y_true-y_pred) #absolute difference between correct and predicted values
        correct = K.less(diff,0.01) #tensor with 0 for false values and 1 for true values
        return K.mean(correct) #sum all 1's and divide by the total. 

if __name__=='__main__':
    rospy.init_node("ros_daev_node",anonymous=True)
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    #get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    #get the keras model
    model=args[1]

    #if there's more than two arguments then its decoupled
    if len(args)>2:
        il=ROS_Daev(racecar_name,model,66,200,decoupled=True)
    else:
        il=ROS_Daev(racecar_name,model,66,200)
        
    image_sub=rospy.Subscriber(il.image_topic,Image,il.image_callback)
    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()