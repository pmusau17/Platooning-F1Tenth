#!/usr/bin/env python
from gazebo_msgs.msg import LinkStates, ContactsState
from std_msgs.msg import Float64, String, Header
from std_srvs.srv import Empty
import math
import tf
import copy
import rospy
import numpy as np
import rospkg
import os 



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

        def __init__(self, tf_prefix, num_obstacles,random_seed,log_file, timeout = 2.0,experiment_number=0):
            self.active_collisions = []
            self.timeout = timeout

            # We only care about collisions of the ego car
            self.tf_prefix = tf_prefix
            # get the path to the log files
            rospack = rospkg.RosPack()
            self.collision_file = os.path.join(rospack.get_path('race'),"logs",log_file)
            rospy.logwarn(self.collision_file)

            # counter utiized to evaluate how many crashes occured
            self.count =0 
            self.num_obstacles = num_obstacles
            self.random_seed = random_seed
            self.log_file = log_file
            self.experiment_number = experiment_number
            self.safe = True 
            self.collision_pub = rospy.Publisher(tf_prefix+'/collisions',String,queue_size=1)


        def is_safe(self):
            return self.safe

        def append_collision(self,ob):
            ob_item1 = list(ob.contacts)[0].split("::")[0]
            ob_item2 = list(ob.contacts)[1].split("::")[0]
            if(os.path.exists(self.collision_file)):
                fi = open(self.collision_file, "a")
                fi.write("{}, {}, {}, {}, {}\n".format(self.random_seed,self.num_obstacles,ob_item1,ob_item2,self.experiment_number))
                fi.close()
            else: 
                fi = open(self.collision_file, "w")
                fi.write("{}, {}, {}, {}, {}\n".format(self.random_seed,self.num_obstacles,ob_item1,ob_item2,self.experiment_number))
                fi.close()
            self.safe = False

            
        def callback_func(self,msg):
            newCollisions = []
            nametuples = []
            if len(msg.states) > 0:
                # traverse through each of the collison states and if it doesn't involve the car ignore it
                for s in msg.states:

                    #make sure the racecar prefix is involved in the collision
                    if (self.tf_prefix not in s.collision1_name) and (self.tf_prefix not in s.collision2_name):
                        continue
                    
                    # if the collision is reported twice ignore it
                    if((s.collision1_name,s.collision2_name) in nametuples):
                        isNew = False
                    else:
                        isNew = True
                    

                    # we use collision objects to handle collisions, if this collision is already in there we update the collison  
                    for coll in self.active_collisions:
                        # True means it was updated, false means it wasn't
                        if coll.update(s,msg.header.stamp):
                            isNew = False
                            break

                    # if it isn't in the list it's new and we can add it to the list
                    if isNew:
                        print((s.collision1_name,s.collision2_name))
                        newCollisions.append(self.Collision(s,msg.header.stamp))
                        self.count+=1
                        self.append_collision(self.Collision(s,msg.header.stamp))
                        nametuples.append((s.collision1_name,s.collision2_name))
                        self.collision_pub.publish("COLLISION %s %s, count: %s" %(s.collision1_name,s.collision2_name,self.count))
        
                        
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
    num_obstacles = args[1]
    random_seed = args[2]
    log_file = args[3]
    experiment_number = args[4:]
    if(experiment_number):
        experiment_number = experiment_number[0]
    rospy.sleep(1)
    CollTracker = CollisionTracker(racecar_name,num_obstacles,random_seed,log_file,experiment_number=experiment_number)
    contact_sub = rospy.Subscriber(racecar_name+'/contact_link_collisions',ContactsState,CollTracker.callback_func,queue_size=100)
    
    r = rospy.Rate(80)
    while CollTracker.is_safe():
        r.sleep()

