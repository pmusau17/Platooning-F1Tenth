#!/usr/bin/env python
from gazebo_msgs.msg import ContactsState
import rospy
import copy
import numpy as np
from std_srvs.srv import Empty

""" This node will listen to the contact sensor and report when the car crashes into another object
    There is also an option to reset the environment for reinforcement learning experiments
"""

class ContactSubscriber:


    # constructor takes two parameters, the name of the racecar
    # whether or not to reset the car on a crash event
    def __init__(self,racecar_name,reset_on_crash=False):
        # need to make sure that the services are correct
        rospy.wait_for_service('/gazebo/reset_world')

        self.reset_on_crash = reset_on_crash

        # counter utiized to evaluate how many crashes occured
        self.count =0 

        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        self.contact_sub = rospy.Subscriber(racecar_name+'/contact_link_collisions',ContactsState,self.callback,queue_size=5)

    def callback(self,msg):
        states=msg.states
        if(states):
            self.count+=1
            rospy.logwarn("Collision Occured")
            rospy.logwarn("Crash Count: {}".format(self.count))
            if(self.reset_on_crash):
                self.reset_proxy()


if __name__=="__main__": 

    rospy.init_node("collision_node")
    args = rospy.myargv()[1:]
    
    # get the racecar name
    
    racecar_name=args[0].replace('/','')

    # reset world 
    reset_world = args[1:]

    if(reset_world):
    # create object 
        cs = ContactSubscriber(racecar_name,reset_on_crash=True)
        rospy.logwarn("Will Reset On Crash")
    else:
        cs = ContactSubscriber(racecar_name)

    rospy.spin()






