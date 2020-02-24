#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import imutils
import matplotlib.pyplot as plt

#this is for live plotting
import matplotlib.animation as animation

#import the tensorflow package
from tensorflow.python.keras.models import load_model
import tensorflow.keras.backend as K
import time

#import the preprocessing utils (helps with loading data, preprocessing)
from preprocessing.utils import ImageUtils
import rospkg

class Analyze_Discrete:
    def __init__(self,racecar_name,model,vesc_name):
        self.cv_bridge=CvBridge()

        #synchronize the input teleop messages with the camera messages
        self.image_rect_color=Subscriber(str(racecar_name)+'/camera/zed/rgb/image_rect_color',Image)
        self.ackermann_stamped=Subscriber(str(vesc_name)+'/ackermann_cmd_mux/input/teleop',AckermannDriveStamped)
        
        #load the keras model
        self.model=load_model(model)
        #this handles the reshaping
        self.util=ImageUtils()      
        #get the height and width from the model so we don't get annoying errors
        self.height=self.model.layers[0].input_shape[1]
        self.width=self.model.layers[0].input_shape[2]

        #image classes
        self.classes=['left','right','straight','weak_left','weak_right']


        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.image_rect_color,self.ackermann_stamped], queue_size = 20, slop = 0.049)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

        #fields for plotting
        self.commands=[]
        self.times=[]
        #ground truth commands
        self.gt_commands=[]
        self.start_time=time.time()
        #figure for live animation
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.window=4000

        #Animation
        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(self.commands, self.times), interval=1000)
        plt.show()

        #master callback for this class
    def master_callback(self,ros_image,ack):
        #preprocess the image
        try:
            orig_image=self.cv_bridge.imgmsg_to_cv2(ros_image,"bgr8")/255.0
            save_img=self.cv_bridge.imgmsg_to_cv2(ros_image,"bgr8")
            cv_image=self.util.reshape_image(orig_image,self.height,self.width)
        except CvBridgeError as e:
            print(e)

        #expand the dimensions so that the model can make a prediction
        predict_image=np.expand_dims(cv_image, axis=0)
        #make the prediction
        pred=self.model.predict(predict_image)

        #get the label and convert it to the respective actuation command
        lb=self.classes[pred[0].argmax()]
        act_command=self.translate_to_actuation_angle(lb)
        self.commands.append(act_command)
        steering_command=ack.drive.steering_angle
        self.gt_commands.append(steering_command)
        self.times.append(time.time()-self.start_time)

        #display the images
        #cv2.imshow("Original Image",orig_image)
        #cv2.imshow("Image passed to network",cv_image)
        #cv2.waitKey(3) 

    #function that animates the plotting
    def animate(self,i,commands,times):
        # Limit x and y lists to window items
        self.commands = self.commands[-self.window:]
        self.times = self.times[-self.window:]
        self.gt_commands=self.gt_commands[-self.window:]
        # Draw x and y lists
        self.ax.clear()
        self.ax.plot(self.times, self.commands,label='Discrete MiniVGG')
        self.ax.plot(self.times, self.gt_commands,label='Continous Wall Following')
        self.ax.legend(loc='upper left')

        # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)
        plt.title('Time (s) vs Steering Angle (radians)')
        plt.ylabel('Steering Angle (radians)')
        plt.xlabel('Time (s)')


    def translate_to_actuation_angle(self,label):
        if (label=="left"):
            angle=0.4108652353
        elif (label=="right"):
            angle=-0.4108652353
        elif (label=="straight"):
            angle=0.0
        elif (label=="weak_left"):
            angle=0.10179
        elif (label=="weak_right"):
            angle=-0.10179
        else: 
            print("error:",label)
            angle=0
        return angle


if __name__=='__main__':
    rospy.init_node("analyze_discrete_node",anonymous=True)
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    #get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    #get the keras model
    model=args[1]
    #get the vescname
    vesc=args[2]
    #create the Analyze End-to-End Model
    ae2e=Analyze_Discrete(racecar_name,model,vesc)

    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()
