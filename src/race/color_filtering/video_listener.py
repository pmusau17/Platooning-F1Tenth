#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 

bridge=CvBridge()



#Helper functions: 

def filter_color(rgb_image,lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image=cv2.cvtColor(rgb_image,cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv image",hsv_image)
    #define a mask using the lower and upper bounds of the yellow color 
    mask=cv2.inRange(hsv_image,lower_bound_color,upper_bound_color)
    #cv2.imshow("mask",mask)

    return mask

def getContours(binary_image):
    _, contours, hierarchy=cv2.findContours(binary_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours



def get_contour_center(contour):
    M=cv2.moments(contour)
    cx=-1 
    cy=-1
    if(M['m00']!=0):
        cx=int(M['m10']/M['m00'])
        cy=int(M['m01']/M['m00'])
    return cx, cy

def draw_ball_contour(binary_image,rgb_image, contours):
    black_image=np.zeros((binary_image.shape[0],binary_image.shape[1],3),'uint8')

    for c in contours:
        #print(c)
        area= cv2.contourArea(c)
        perimeter=cv2.arcLength(c,True)
        ((x,y),radius)=cv2.minEnclosingCircle(c)
        #This eliminates noisy contour detectors
        if(area>3000):
            cv2.drawContours(rgb_image,[c],-1,(150,250,150),1)
            cv2.drawContours(black_image,[c],-1,(150,250,150),1)
            cx, cy=get_contour_center(c)
            cv2.circle(rgb_image,(cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image,(cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image,(cx,cy),5,(150,150,255),-1)
            print("Area: {}, Perimeter:{}".format(area,perimeter))
    print("number of contours: {}".format(len(contours)))
    cv2.imshow("RGB Image Contours",rgb_image)
    cv2.imshow("Black Image Contours",black_image)

def image_callback(ros_image):
    print("got an image")
    #define the upper and lower bounds of the color you are trying to detect
    yellowLower=(30,150,100)
    yellowUpper=(50,255,255)
    global bridge
    #convert ros_image into an opencv-compatible image
    try:
        cv_image=bridge.imgmsg_to_cv2(ros_image,"bgr8")
    except CvBridgeError as e:
        print(e)
    (rows,cols,channels)=cv_image.shape
    if cols>200 and rows>200:
        binary_image=filter_color(cv_image,yellowLower,yellowUpper)
        rgb_contours=getContours(binary_image)
        draw_ball_contour(binary_image,cv_image,rgb_contours)
        #cv2.imshow("Received Image",cv_image)
        cv2.waitKey(3)

def main():
    rospy.init_node("image_converter",anonymous=True)
    image_sub=rospy.Subscriber("/tennis_ball_image",Image,image_callback)
    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()
