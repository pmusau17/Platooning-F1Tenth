#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty

rospy.init_node('reset_world')

rospy.wait_for_service('/gazebo/reset_world')
rospy.wait_for_service('/gazebo/pause_physics')
pause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

reset_world()
#pause()