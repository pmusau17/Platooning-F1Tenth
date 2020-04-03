#!/usr/bin/env python
import rospy
import cv2
from race.msg import prediction
from race.msg import drive_param
import numpy as np 

#need to subscribe to the steering message and angle message
from message_filters import ApproximateTimeSynchronizer, Subscriber

class EnsembleManager:
    # this will subscribe to all the predictions 
    # given by the individual models
    def __init__(self,racecar_name,model_names):
        self.racecar_name=racecar_name

        # SLOPE 
        SLOPE =0.049
        # models is a list of names
        # for each name create the subscriber
        model_subs = []

        # exit if no models are passed
        if len(model_names)==0:
            raise RuntimeError("You didn't pass in any models as arguments")

        for name in model_names:
            model_subs.append(Subscriber(racecar_name+'/'+name+'/pred',prediction)) 


        # define the classes
        self.classes=['left','right','straight','weak_left','weak_right']

        # create the publisher
        self.pub=rospy.Publisher(self.racecar_name+'/drive_parameters',drive_param,queue_size=20)

        #create the time synchronizer
        self.sub = ApproximateTimeSynchronizer(model_subs, queue_size = 20, slop = SLOPE)

        #register the callback to the synchronizer
        self.sub.registerCallback(self.master_callback)
    
    #depending on the number of models you need variable arguments
    def master_callback(self,*args):
        # get the predictions 
        preds=[]
        for msg in args:
            preds.append(np.asarray(msg.prediction))
        preds = np.asarray(preds)

        # average the predictions
        averages = np.average(preds,axis=0)

        #print(preds)
        self.send_actuation_command(averages)
        

    #computes the actuation command to send to the car
    def send_actuation_command(self,averages):
        #create the drive param message
        msg = drive_param()
        msg.header.stamp = rospy.Time.now()
        msg.angle = 0.0
        msg.velocity = 1.0
        #get the label
        label=self.classes[averages.argmax()]
        print("[INFO] pred:",label)
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
        self.pub.publish(msg)


if __name__=="__main__":
    #get the arguments passed from the launch file 
    args = rospy.myargv()[1:]
    racecar_name=args[0]
    racecar_models=args[1:]
    rospy.init_node('decision_manager_'+racecar_name.replace('/',''), anonymous=True)
    dm = EnsembleManager(racecar_name,racecar_models)
    rospy.spin()