#!/usr/bin/env python
import rospy
import cv2
from race.msg import prediction
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
from sensor_msgs.msg import Image

#import the tensorflow package
from tensorflow.python.keras.models import load_model

#import the preprocessing utils (helps with loading data, preprocessing)
from preprocessing.utils import ImageUtils

class RosEnsembleNode:
    #define the constructor 
    def __init__(self,racecar_name,model,name):
        self.cv_bridge=CvBridge()
        self.image_topic=str(racecar_name)+'/camera/zed/rgb/image_rect_color'
        self.model=load_model(model)
        #this handles the reshaping
        self.util=ImageUtils()

        #get the image shape from the model 
        self.height=self.model.layers[0].input_shape[1]
        self.width=self.model.layers[0].input_shape[2]

        # display classes for printing purposes
        self.classes=['left','right','straight','weak_left','weak_right']

        self.pub=rospy.Publisher(racecar_name+'/'+name+'/pred', prediction, queue_size=5)
        self.image_sub=rospy.Subscriber(self.image_topic,Image,self.image_callback)

    #image callback
    def image_callback(self,ros_image):
        #convert the ros_image to an openCV image
        try:
            orig_image=self.cv_bridge.imgmsg_to_cv2(ros_image,"bgr8")/255.0
            cv_image=self.util.reshape_image(orig_image,self.height,self.width)
            #print(cv_image.shape)
        except CvBridgeError as e:
            print(e)

        #reshape the image so we can feed it ot the neural network
        predict_image=np.expand_dims(cv_image, axis=0)
        #make the prediction
        pred=self.model.predict(predict_image)
        #publish the actuation command
        
        msg = prediction()
        msg.header.stamp = rospy.Time.now()
        msg.prediction = pred[0]
        self.pub.publish(msg)

        #uncomment the following to display the image classification
        #cv2.imshow(self.classes[pred[0].argmax()],predict_image[0])
        
        #log the result to the console
        print("[INFO] prediction: {}".format(self.classes[pred[0].argmax()]))


if __name__=='__main__':
    rospy.init_node("classify_node",anonymous=True)
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    #get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    # get the keras model
    model=args[1]
    # you need a name so that the decision manager knows what to subscribe to
    name = args[2]

    ensemble_node=RosEnsembleNode(racecar_name,model,name)
    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()