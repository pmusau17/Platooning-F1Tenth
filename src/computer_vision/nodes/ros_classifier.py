#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import imutils
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

# sometimes GPU systems are annoying 
from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import Session
config = ConfigProto()
config.gpu_options.allow_growth = True
sess = Session(config=config)
sess.as_default()

# import the custom message files defined the race package
from race.msg import drive_param
from race.msg import angle_msg

#import the tensorflow package
from tensorflow.python.keras.models import load_model

# import sys so we can use packages outside of this folder in
# either python 2 or python 3, I know it's janky, chill
import sys
import os
from pathlib import Path 
#insert parent directory into the path
sys.path.insert(0,str(Path(os.path.abspath(__file__)).parent.parent))

#import the preprocessing utils (helps with loading data, preprocessing)
from preprocessing.utils import ImageUtils


class ROS_Classify:

    #define the constructor 
    def __init__(self,racecar_name,model,decoupled=False,plot=False):


        self.cv_bridge=CvBridge()
        self.image_topic=str(racecar_name)+'/camera/zed/rgb/image_rect_color'
        self.model=load_model(model)

        self.count = 0
        self.util=ImageUtils()
        # depends how the model was trained
        try:
            self.height=self.model.layers[0].input_shape[1]
            self.width=self.model.layers[0].input_shape[2]
        except IndexError as e:
            self.height=self.model.layers[0].input_shape[0][1]
            self.width=self.model.layers[0].input_shape[0][2]

        self.classes=['left','right','straight','weak_left','weak_right']

        self.decoupled=decoupled
        if (not self.decoupled):
            self.pub=rospy.Publisher(racecar_name+'/drive_parameters', drive_param, queue_size=5)
        else:
            self.pub=rospy.Publisher(racecar_name+'/angle_msg',angle_msg,queue_size=5)

        self.image_sub=rospy.Subscriber(self.image_topic,Image,self.image_callback)
        
        self.plot = plot
        if self.plot:
            #fields for plotting
            self.commands=[]
            self.times=[]
            self.start_time=time.time()
            #figure for live animation
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(1, 1, 1)
            self.window=4000


            #Animation
            # Set up plot to call animate() function periodically
            ani = animation.FuncAnimation(self.fig, self.animate, fargs=(self.commands, self.times), interval=1000)
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
        self.count+=1
        #publish the actuation command
        if(self.count>20):
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
        #get the label
        label=self.classes[pred[0].argmax()]
        if (label=="left"):
            angle=0.6108652381980153
        elif (label=="right"):
            angle=-0.6108652381980153
        elif (label=="straight"):
            angle=0.0
        elif (label=="weak_left"):
            angle=0.17453292519943295
        elif (label=="weak_right"):
            angle=-0.17453292519943295
        else: 
            print("error:",label)
            angle=0

        if (self.plot):
            self.commands.append(angle)
            self.times.append(time.time()-self.start_time)

        if (not self.decoupled):
            msg = drive_param()
            msg.header.stamp=rospy.Time.now()
            msg.angle = angle
            msg.velocity = 1.0
        else:
            msg=angle_msg()
            msg.header.stamp=rospy.Time.now()
            msg.steering_angle=angle
        self.pub.publish(msg)

    #function that animates the plotting
    def animate(self,i,commands,times):
        # Limit x and y lists to window items
        self.commands = self.commands[-self.window:]
        self.times = self.times[-self.window:]
        # Draw x and y lists
        self.ax.clear()
        self.ax.plot(self.times, self.commands)

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


    #if there's more than two arguments then its decoupled
    if len(args)>2:
        il=ROS_Classify(racecar_name,model,decoupled=True)
    else:
        il=ROS_Classify(racecar_name,model)
    try: 
        r = rospy.Rate((1/40.0))
        while not rospy.is_shutdown():
            r.sleep()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()