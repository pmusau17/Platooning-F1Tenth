#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import signal 
import sys


#This function is the handler for ctrl-c so that this script quits nicely
def signal_handler(sig, frame):
        print('kiled node')
        cv2.destroyAllWindows()
        sys.exit(0)
#define the signal listener
signal.signal(signal.SIGINT, signal_handler)

#ROS portion
bridge=CvBridge()
pub=rospy.Publisher("tennis_ball_image",Image,queue_size=10)
rospy.init_node("video_publisher",anonymous=True)
video_capture=cv2.VideoCapture('/home/musaup/Documents/Research/DroneChallenge/images/tennis-ball-video.mp4')
r = rospy.Rate(10) # 10hz
if __name__=="__main__":
    try:
        while(True):
            ret,frame=video_capture.read()
            print(frame.shape)
            cv_image=bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(cv_image)
            if 0xFF==ord('q'): #this quits if you press the key 'q'
                break
            r.sleep()
    except (CvBridgeError,KeyboardInterrupt) as e:
        print("Caught")
