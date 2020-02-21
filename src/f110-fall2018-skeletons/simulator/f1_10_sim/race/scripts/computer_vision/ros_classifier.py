#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from race.msg import drive_param
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import imutils
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

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
        self.pub=rospy.Publisher(racecar_name+'/drive_parameters', drive_param, queue_size=5)

        #fields for plotting
        self.commands=[]
        self.time=[]
        self.start_time=time.time()
        #figure for live animation
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.window=4000

        #Animation
        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(self.fig, self.animate, interval=200)
        plt.show()

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
        self.send_actuation_command(pred)

        #uncomment the following to display the image classification
        #cv2.imshow(self.classes[pred[0].argmax()],predict_image[0])
        
        #log the result to the console
        print("INFO prediction: {}".format(self.classes[pred[0].argmax()]))
        
        #uncomment to show the original image
        #cv2.imshow("Original Image",orig_image)
        #cv2.waitKey(3) 

    #computes the actuation command to send to the car
    def send_actuation_command(self,pred):
        #create the drive param message
        msg = drive_param()
        msg.angle = 0.0
        msg.velocity = 1.0
        #get the label
        label=self.classes[pred[0].argmax()]

        if (label=="left"):
            msg.angle=0.4108652353
        elif (label=="right"):
            msg.angle=-0.4108652353
        elif (label=="straight"):
            msg.angle=0.0
        elif (label=="weak_left"):
            msg.angle=0.10179
        elif (label=="weak_right"):
            msg.angle=-0.10179
        else: 
            print("error:",label)
            msg.velocity=0
            msg.angle=0
        self.commands.append(msg.angle)
        self.time.append(time.time()-self.start_time)
        self.pub.publish(msg)

    #function that animates the plotting
    def animate(self,i, buckets, misclassifications):
        # Limit x and y lists to window items
        self.commands = self.commands[-self.window:]
        self.time = self.time[-self.window:]

        # Draw x and y lists
        self.ax.clear()
        self.ax.plot(self.time, self.commands)

        # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)
        plt.title('Time (s) vs Steering Angle (radians)')
        plt.ylabel('Steering Angle (radians)')
        plt.xlabel('Time (s)')


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