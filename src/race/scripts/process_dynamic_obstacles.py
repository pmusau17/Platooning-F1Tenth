#!/usr/bin/env python
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import rospy
import math
import rospkg
import os
#need to subscribe to the steering message and angle message
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np 

class ProcessDynamicObstacles:
    def __init__(self,racecar_name):
        self.racecar_name = racecar_name
        self.pub = rospy.Publisher(self.racecar_name+"/processed_obstacles", MarkerArray, queue_size="1")

        #subscribe to the viz and the odom message
        self.marker_sub=Subscriber('viz',MarkerArray)
        self.odom_sub=Subscriber(self.racecar_name+'/odom',Odometry)
        self.sub = ApproximateTimeSynchronizer([self.marker_sub,self.odom_sub], queue_size = 20, slop = 0.019,allow_headerless=True)

        self.sub.registerCallback(self.master_callback)

    def calculate_intervals(self,center,width=1,height=1):
        '''
            Addition by Patrick for RealTime Reachability
        '''
        x_int = [center[0]-width/2, center[0]+width/2]
        y_int = [center[1]-height/2, center[1]+height/2]
        return x_int,y_int 

    def master_callback(self,marker_array_msg,odom_msg):
        # position 
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        odom_point = np.asarray([x,y]).reshape((1,2))

        # Convert Quaternion to rpy
        rpy = euler_from_quaternion([odom_msg.pose.pose.orientation.x,
                                     odom_msg.pose.pose.orientation.y,
                                     odom_msg.pose.pose.orientation.z,
                                     odom_msg.pose.pose.orientation.w])

        # markers
        markerArray = MarkerArray()
        markers = marker_array_msg.markers
        for marker in markers:
            mx = marker.pose.position.x
            my = marker.pose.position.y
            
            marker_point = np.asarray([mx,my]).reshape((1,2))
            diff = np.linalg.norm(odom_point - marker_point)

            # Ignore obstacles that are more than 5.0 meters away, 
            # Change this if needed
            if(diff<5.0):
                marker.id = marker.id+1
                marker.lifetime = rospy.Duration(0.05)
                markerArray.markers.append(marker)
                xi,yi = self.calculate_intervals((mx,my),width=marker.scale.x,height=marker.scale.y)
                print(xi,yi)
                

        self.pub.publish(markerArray)




if __name__=="__main__":
    #get the arguments passed from the launch file 
    args = rospy.myargv()[1:]
    racecar_name=args[0]
    rospy.init_node('decision_manager_'+racecar_name, anonymous=True)
    dm =  ProcessDynamicObstacles(racecar_name)
    rospy.spin()








#get the arguments passed from the launch file
args = rospy.myargv()[1:]

# get the path to the file containing the waypoints
waypoint_file=args[0]

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
#get the path for this paackage
package_path=rospack.get_path('pure_pursuit')

filename=os.path.sep.join([package_path,'waypoints',waypoint_file])
with open(filename) as f:
	path_points = [tuple(line) for line in csv.reader(f)]

topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size="1")
rospy.init_node('register')

# Visualize every other marker to save on memory and speed
while not rospy.is_shutdown():
        markerArray = MarkerArray()
        for i in range(len(path_points)):
                if i % 2 == 0:
                        point = path_points[i]
		        x = float(point[0])
		        y = float(point[1])
		        marker = Marker()
		        marker.header.frame_id = "/map"
		        marker.type = marker.SPHERE
		        marker.action = marker.ADD
		        marker.scale.x = 0.1
		        marker.scale.y = 0.1
		        marker.scale.z = 0.1
		        marker.color.a = 1.0
		        marker.color.r = 1.0
		        marker.color.g = 1.0
		        marker.color.b = 0.0
		        marker.pose.orientation.w = 1.0
		        marker.pose.position.x = x
		        marker.pose.position.y = y
		        marker.pose.position.z = 0

		        markerArray.markers.append(marker)
        
	# Renumber the marker IDs
	id = 0
	for m in markerArray.markers:
		m.id = id
		id += 1

	# Publish the MarkerArray
	publisher.publish(markerArray)

	rospy.sleep(5.0)
