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
import datetime as dt

#import the tensorflow package
from tensorflow.python.keras.models import load_model
import tensorflow.keras.backend as K

#import the preprocessing utils (helps with loading data, preprocessing)
from preprocessing.utils import ImageUtils
import rospkg

""" This class evaluates the end to end model with respect to the disparity extender
    The assumptions was that the disparity extender was the expert. This helps collect 
    data in areas that we see the car performing poorly.
"""

class AnalyzeE2E:


    def __init__(self,racecar_name,model,vesc_name):
        self.cv_bridge=CvBridge()

        #synchronize the input teleop messages with the camera messages
        self.image_rect_color=Subscriber(str(racecar_name)+'/camera/zed/rgb/image_rect_color',Image)
        self.ackermann_stamped=Subscriber(str(vesc_name)+'/ackermann_cmd_mux/input/teleop',AckermannDriveStamped)
        
        #to save the image data get the path
        r = rospkg.RosPack()
        self.save_path_root=r.get_path('race')+'/scripts/computer_vision/data/'

        #load the keras model
        self.model=load_model(model,custom_objects={'customAccuracy': self.customAccuracy})
        #this handles the reshaping
        self.util=ImageUtils()      
        #get the height and width from the model so we don't get annoying errors
        self.height=self.model.layers[0].input_shape[1]
        self.width=self.model.layers[0].input_shape[2]
        
        #window that we track
        self.window=200

        #tolerance
        self.tolerance=0.02

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.image_rect_color,self.ackermann_stamped], queue_size = 20, slop = 0.049)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

        #this is for plotting
        self.xs=[]
        self.ys=[]
        self.count=0
        # Create figure for plotting
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)

        #Animation
        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(self.xs, self.ys), interval=1000)
        plt.show()
        
    

    #master callback for this class
    def master_callback(self,ros_image,ack):
        try:
            orig_image=self.cv_bridge.imgmsg_to_cv2(ros_image,"bgr8")/255.0
            cv_image=self.util.reshape_image(orig_image,self.height,self.width)
            save_img=self.cv_bridge.imgmsg_to_cv2(ros_image,"bgr8")
            #print(cv_image.shape)
        except CvBridgeError as e:
            print(e)

        predict_image=np.expand_dims(cv_image, axis=0)
        pred=self.model.predict(predict_image)[0]*0.6108652353

        #get the angle from the ackermann message

        #get the difference need it to be smaller than 0.01 radians which 0.57 degrees we can make it smaller but 
        #that is fine for now
        diff=abs(pred[0]-ack.drive.steering_angle)
        #self.xs.append("%.2f" % float(dt.datetime.now().strftime('%S.%f')))
        self.xs.append(self.count)
        self.ys.append(diff)
        self.count+=1
        
        if diff>self.tolerance:
            command='%.10f' % ack.drive.steering_angle
            #replace the period with ~ so it's a valid filename
            command=command.replace('.','~')
            save_path=self.save_path_root+self.label_image(ack.drive.steering_angle)+'/'+str(rospy.Time.now())+'~'+command+'.jpg'
            print(self.label_image(ack.drive.steering_angle))
            cv2.imwrite(save_path,save_img)

    def animate(self,i, xs, ys):
        # Limit x and y lists to window items
        self.ys = self.ys[-self.window:]
        self.xs = self.xs[-self.window:]

        # Draw x and y lists
        self.ax.clear()
        self.ax.plot(self.xs, self.ys)

        # Format plot
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)
        plt.title('Difference between prediction and Ground Truth')
        plt.ylabel('Error')
        plt.xlabel('index of prediction')


    #this is used for saving it to correct directory
    #function that categorizes images into left, weak_left, straight, weak_right, right
    def label_image(self,steering_angle):
        if(steering_angle<-0.20179):
            return "right"
        elif(steering_angle>0.20179):
            return "left"
        elif(steering_angle<-0.0523599 and steering_angle>-0.20179):
            return "weak_right"
        elif(steering_angle>0.0523599 and steering_angle<0.20179):
            return "weak_left"
        else:
            return "straight"



    #define a custom metric for DAEV, accuracy doesn't cut it
    def customAccuracy(self,y_true, y_pred):
        diff = K.abs(y_true-y_pred) #absolute difference between correct and predicted values
        correct = K.less(diff,0.01) #tensor with 0 for false values and 1 for true values
        return K.mean(correct) #sum all 1's and divide by the total. 


if __name__=='__main__':
    rospy.init_node("analyze_e2e_node",anonymous=True)
    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    #get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    #get the keras model
    model=args[1]
    #get the vescname
    vesc=args[2]
    #create the Analyze End-to-End Model
    ae2e=AnalyzeE2E(racecar_name,model,vesc)

    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()