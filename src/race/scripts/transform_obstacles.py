#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import copy

class ObstacleTransformer:
    def __init__(self, *args):
        self.tfl = tf.TransformListener()
        self.visPub = rospy.Publisher('viz2', MarkerArray,queue_size="1")

    def viz_callback(self,data):
        markerArray = MarkerArray()
        num_markers = len(data.markers)
        count = 10000
        if self.tfl.frameExists("racecar2/laser") and self.tfl.frameExists("racecar2/odom"):
            for i in data.markers:
                try:
                    self.tfl.clear()
                    p1 = PoseStamped()
                    p1.header.frame_id = i.header.frame_id
                    self.tfl.waitForTransform("racecar2/odom", "racecar2/laser", rospy.Time(0),rospy.Duration(0.05))
                    p1.header.stamp = self.tfl.getLatestCommonTime("racecar2/laser", "racecar2/odom")
                    p1.pose = i.pose
                    p1.pose.orientation.w = 1.0
                    p_in_base = self.tfl.transformPose("racecar2/odom", p1)

                    marker = copy.deepcopy(i)
                    marker.id = num_markers+count
                    marker.pose = p_in_base.pose
                    marker.pose.orientation.w = 1.0
                    marker.header.frame_id = 'map'
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    markerArray.markers.append(marker)
                    count+=1
                except Exception as e:
                    print(e)
        self.visPub.publish(markerArray)

if __name__ == '__main__':
    rospy.init_node('obstacle_transformer', anonymous=True)
    OT=ObstacleTransformer()
    #wait three seconds so that the simulation sets up properly
    rospy.Subscriber('viz', MarkerArray, OT.viz_callback)
    rospy.spin()