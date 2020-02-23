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

#import the preprocessing utils (helps with loading data, preprocessing)
from preprocessing.utils import ImageUtils
import rospkg

""" This class evaluates the classification model. Based on the way we discretize the data,
    i.e how many classes, check how the model is performing at runtime. Again here the data
    was collected using the disparity extender but we can swap the driving algorithm as 
    we wish.
"""

class AnalyzeClassification:
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

        #to save the image data get the path
        r = rospkg.RosPack()
        self.save_path_root=r.get_path('race')+'/scripts/computer_vision/data/'
        self.count=0

        #image classes
        self.classes=['left','right','straight','weak_left','weak_right']

        #this will be used to display misclassifications
        self.buckets=np.arange(len(self.classes))
        #misclassification counts
        self.misclassifications=np.zeros(len(self.classes))

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer([self.image_rect_color,self.ackermann_stamped], queue_size = 20, slop = 0.049)
        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)

        #figure for live animation
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.ls=('left','right','straight','weak_left','weak_right')

        


        #Animation
        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(self.fig, self.animate, fargs=(self.buckets, self.misclassifications), interval=1000)
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

        #get the label and ground truth
        lb=self.classes[pred[0].argmax()]
        gt=self.label_image(ack.drive.steering_angle)

        #create the text to show on the openCv Image
        label='prediction: '+self.classes[pred[0].argmax()]
        #get the ground truth label
        ground_truth_label='ground_truth: '+self.label_image(ack.drive.steering_angle)

        if(lb!=gt):
            #get the index of the misclassification
            ind=self.classes.index(lb)
            #increment our counter
            self.misclassifications[ind]+=1
            print(self.misclassifications,lb)

            command='%.10f' % ack.drive.steering_angle
            #replace the period with ~ so it's a valid filename
            command=command.replace('.','~')
            save_path=self.save_path_root+self.label_image(ack.drive.steering_angle)+'/'+str(rospy.Time.now())+'~'+command+'.jpg'
            if self.label_image(ack.drive.steering_angle)!='straight':
                cv2.imwrite(save_path,save_img)
                cv2.imshow("Original Image",save_img)
                cv2.waitKey(25)
                print(lb)
            else: 
                print("nah its straight")
        
        #cv2.putText(orig_image, label, (32, 32),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        #cv2.putText(orig_image, ground_truth_label, (32, 460),cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 255), 2)
        
        #if(self.count%10==0):
         #   cv2.imshow("Original Image",orig_image)
          #  cv2.waitKey(25) 
        self.count+=1
    #this was how the training data was collected
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

    #function that animates the matplotlib
    def animate(self,i, buckets, misclassifications):
        # Draw x and y lists
        self.ax.clear()
        self.ax.bar(self.buckets, self.misclassifications, alpha=0.5)

        # Format plot
        #plt.subplots_adjust(bottom=0.30)
        plt.xticks(self.buckets, self.ls)
        plt.ylabel('Number of Misclassifications')
        plt.title('Classification Model Performance')



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
    ae2e=AnalyzeClassification(racecar_name,model,vesc)

    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
        cv2.destroyAllWindows()