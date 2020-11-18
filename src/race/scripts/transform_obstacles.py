#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class ObstacleTransformer:
    def __init__(self, *args):
        self.tfl = tf.TransformListener()
        self.visPub = rospy.Publisher('viz2', MarkerArray,queue_size="10")

    def viz_callback(self,data):
        markerArray = MarkerArray()
        if self.tfl.frameExists("racecar2/laser") and self.tfl.frameExists("racecar2/odom"):
            for i in data.markers:
                p1 = PoseStamped()
                p1.header.frame_id = i.header.frame_id
                p1.header.stamp = self.tfl.getLatestCommonTime("/racecar2/laser", "racecar2/odom")
                p1.pose = i.pose
                p1.pose.orientation.w = 1.0 
                p_in_base = self.tfl.transformPose("/racecar2/odom", p1)
                i.pose = p_in_base.pose
                i.header.frame_id = '/map'
                markerArray.markers.append(i)
                self.visPub.publish(markerArray)

if __name__ == '__main__':
    rospy.init_node('obstacle_transformer', anonymous=True)
    OT=ObstacleTransformer()
    #wait three seconds so that the simulation sets up properly
    rospy.Subscriber('viz', MarkerArray, OT.viz_callback)
    rospy.spin()