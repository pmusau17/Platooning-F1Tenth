#!/usr/bin/env python 

import rospy 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal,MoveBaseGoal

waypoints=[(2.0,0.0,0.0,1.0),(4.0,0.0,0.0,1.0),(6.0,0.0,0.0,1.0),(8.16506671906,-0.197142720222,-0.0687395704375,0.99763463826),
(10.0329904556,-0.0784645080566,0.301981188558,0.953313884173),(10.4943914413,1.95681345463,0.692681969459,0.721243155382),
(10.3213567734, 5.10574102402, 0.700949700809,0.713210710054), (9.11966896057, 8.55480957031, 0.998222784385, 0.0595925560301),(7.11966896057, 8.55480957031, 0.998222784385, 0.0595925560301),
(5.11966896057, 8.55480957031, 0.998222784385, 0.0595925560301),(3.11966896057, 8.55480957031, 0.998222784385, 0.0595925560301),(1.11966896057, 8.55480957031, 0.998222784385, 0.0595925560301),
(0.775521278381,8.59584522247,0.999781353896,0.0209103898119),(-1.775521278381,8.59584522247,0.999781353896,0.0209103898119),(-3.775521278381,8.59584522247,0.999781353896,0.0209103898119),(-5.775521278381,8.59584522247,0.999781353896,0.0209103898119),(-7.775521278381,8.59584522247,0.999781353896,0.0209103898119),(-8.775521278381,8.59584522247,0.999781353896,0.0209103898119),(-10.8269100189,8.81981945038,0.989605546881,0.143808419721),(-11.7141876221,8.92541694641, 0.995061838111,-0.0992569309146),
(-12.3620681763,8.40375995636,-0.795801866187,0.605557090433),
(-12.3620681763,5.40375995636,-0.795801866187,0.605557090433),(-12.4704837799, 4.99882507324,-0.702899619951,0.711289058171),(-12.4704837799, 1.99882507324,-0.702899619951,0.711289058171),(-12.4332790375, 0.832118988037, -0.426622896216,0.904429601696),(-11.08777809143, 0.127003669739,0.000382682231288,0.999999926777),(-9.08777809143, 0.127003669739,0.000382682231288,0.999999926777),(-7.08777809143, 0.127003669739,0.000382682231288,0.999999926777),(-5.08777809143, 0.127003669739,0.000382682231288,0.999999926777),(-3.08777809143, 0.127003669739,0.000382682231288,0.999999926777),(-1.08777809143, 0.127003669739,0.000382682231288,0.999999926777),(0, 0,0.000382682231288,0.999999926777)]



def go_to_goal(x,y,z,w):
    global client
    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id="map"
    goal.target_pose.header.stamp=rospy.Time.now()
    goal.target_pose.pose.position.x=x
    goal.target_pose.pose.position.y=y
    goal.target_pose.pose.orientation.z=z
    goal.target_pose.pose.orientation.w=w

    client.send_goal(goal)
    wait=client.wait_for_result(rospy.Duration(5.0))
    rospy.loginfo(str(wait))
    if not wait:
        rospy.logerr("Action server not available or goal did not complete")
	#The rospy signal shutdown command shuts down a node the argument is a string describing the reason for shutting the node down
        #rospy.signal_shutdown("Action sever not available!")
    else:
        return client.get_result()




def movebase_client():
    global client
    goal=MoveBaseGoal()
    goal.target_pose.header.frame_id="map"
    goal.target_pose.header.stamp=rospy.Time.now()
    goal.target_pose.pose.position.x=2.0
    goal.target_pose.pose.position.y=0.0
    goal.target_pose.pose.orientation.w=1.0

    client.send_goal(goal)
    wait=client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available")
        rospy.signal_shutdown("Action sever not available!")
    else:
        return client.get_result()

if __name__=="__main__":
    try:
        rospy.init_node("movebase_client_py")
        client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        for i in range(len(waypoints)):
            goal=waypoints[i]
            result=go_to_goal(goal[0],goal[1],goal[2],goal[3])
            if result:
                rospy.loginfo("Goal Execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished")
