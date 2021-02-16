#!/usr/bin/env python
from gazebo_msgs.msg import LinkStates, ContactsState
from std_msgs.msg import Float64, String, Header
from std_srvs.srv import Empty
import math
import tf
import copy
import rospy
import numpy as np




class CollisionTracker(object):
        """Object for reducing frequent Gazebo updates to single-event updates

        Gazebo publishes collisions at a very high rate, and simply reports which geometries are 
        intersecting. This is frequently too fast, and sometimes a single message is missed. This 
        class instead stores which collisions have already occurred and only removes them after a 
        timeout.

        Attributes
        ----------
        timeout : float
            The amount of time after which a collision is assumed to have "stopped"
        msg : gazebo_msgs/ContactsState
            The message from Gazebo updating the bumper plugin state

        Methods
        -------
        callback_func(msg)
            The tracker's callback, which should be handed to the subscriber receiving Gazebo msgs

        Developped by Tim Krentz
        """

        def __init__(self, tf_prefix, timeout = 0.5, reset_on_crash=False):
            self.active_collisions = []
            self.timeout = timeout
            self.tf_prefix = tf_prefix
            self.reset_on_crash = reset_on_crash
            
            # If utilized within reinforcement learning, the service is needed to reset the world

            if(self.reset_on_crash):
                rospy.wait_for_service('/gazebo/reset_world')
                self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
               
            # counter utiized to evaluate how many crashes occured
            self.count =0 


            self.collision_pub = rospy.Publisher(tf_prefix+'/collisions',String,queue_size=1)


            
        def callback_func(self,msg):
            newCollisions = []
            if len(msg.states) > 0:
                for s in msg.states:
                    if (self.tf_prefix not in s.collision1_name) and (self.tf_prefix not in s.collision2_name):
                        continue
                    isNew = True
                    for coll in self.active_collisions:
                        # True means it was updated, false means it wasn't
                        if coll.update(s,msg.header.stamp):
                            isNew = False
                            break
                    # if it is new we create a a collision object for it
                    if isNew:
                        newCollisions.append(self.Collision(s,msg.header.stamp))
                        self.count+=1
                        self.collision_pub.publish("COLLISION %s %s, count: %s" %(s.collision1_name,s.collision2_name,self.count))
                        if (self.reset_on_crash):
                            self.reset_proxy()
                        


            for coll in self.active_collisions:
                if (msg.header.stamp-coll.stamp) < rospy.Duration.from_sec(self.timeout):
                    newCollisions.append(copy.deepcopy(coll))
            self.active_collisions = newCollisions

        class Collision(object):
            def __init__(self,state,ts):
                self.contacts = copy.deepcopy(set([state.collision1_name,state.collision2_name]))
                self.stamp = copy.deepcopy(ts)

            def update(self,newState,ts):
                if set([newState.collision1_name,newState.collision2_name]) == self.contacts:
                    self.stamp = copy.deepcopy(ts)
                    return True
                return False


if __name__ == "__main__":
    rospy.init_node('collision_tracker')

    args = rospy.myargv()[1:]
    
    # get the racecar name
    
    racecar_name=args[0].replace('/','')

    # reset world 
    reset_world = args[1:]

    if(reset_world):
    # create object 
        CollTracker = CollisionTracker(racecar_name,reset_on_crash=True)
        rospy.logwarn("Will Reset On Crash")
    else:
         CollTracker = CollisionTracker(racecar_name)


    contact_sub = rospy.Subscriber(racecar_name+'/contact_link_collisions',ContactsState,CollTracker.callback_func,queue_size=100)
    rospy.spin()

