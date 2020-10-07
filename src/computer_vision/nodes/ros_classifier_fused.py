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
from message_filters import ApproximateTimeSynchronizer, Subscriber

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
from sensor_msgs.msg import LaserScan

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
        self.image_topic = racecar_name+'/camera/zed/rgb/image_rect_color'
        self.lidar_topic = racecar_name+'/scan'
        self.image_rect_color=Subscriber(self.image_topic,Image)
        self.lidar_sub=Subscriber(self.lidar_topic,LaserScan)
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

        if(not self.height==32 or not self.width ==32):
            print("Model provided is incompatible, the image size should be 32 x 32 it is",self.height,"x",self.width)
            exit()

        self.classes=['left','right','straight','weak_left','weak_right']

        self.decoupled=decoupled
        if (not self.decoupled):
            self.pub=rospy.Publisher(racecar_name+'/drive_parameters', drive_param, queue_size=5)
        else:
            self.pub=rospy.Publisher(racecar_name+'/angle_msg',angle_msg,queue_size=5)

        self.sub = ApproximateTimeSynchronizer([self.image_rect_color,self.lidar_sub], queue_size = 20, slop = 0.08)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.image_callback)
        
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
    def image_callback(self,ros_image,lidar_msg):
        #convert the ros_image to an openCV image
        try:
            orig_image=self.cv_bridge.imgmsg_to_cv2(ros_image,"bgr8")/255.0
            cv_image=self.util.reshape_image(orig_image,32,32)
            #print(cv_image.shape)
        except CvBridgeError as e:
            print(e)

        limited_ranges=np.asarray(lidar_msg.ranges)
        indices=np.where(limited_ranges>=10.0)[0]
        limited_ranges[indices]=10.0
        limited_ranges= limited_ranges[29:1053]
        limited_ranges = limited_ranges.reshape((32,32,1))
        limited_ranges = limited_ranges

        # concatenate the images with the laser scan
        pred_image = np.concatenate((cv_image,limited_ranges),axis = -1)
        predict_image=np.expand_dims(pred_image, axis=0)
        #make the prediction
        pred=self.model.predict(predict_image)
        self.count+=1
        #publish the actuation command
        if(self.count>20):
            self.send_actuation_command(pred)

        #log the result to the console
        print("INFO prediction: {}".format(self.classes[pred[0].argmax()]))
        
    #computes the actuation command to send to the car
    def send_actuation_command(self,pred):
        #get the label
        label=self.classes[pred[0].argmax()]
        if (label=="left"):
            angle=0.5108652353
        elif (label=="right"):
            angle=-0.5108652353
        elif (label=="straight"):
            angle=0.0
        elif (label=="weak_left"):
            angle=0.17179
        elif (label=="weak_right"):
            angle=-0.17179
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
            msg.velocity = 0.5
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
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()