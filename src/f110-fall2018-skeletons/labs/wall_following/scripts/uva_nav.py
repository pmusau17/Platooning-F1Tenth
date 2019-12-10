"""
Author : Varundev Suresh Babu
Version: 0.1
"""

import rospy
from std_msgs.msg import Float64

steering_publisher = ospy.Publisher("/servo/position", Float64, queue_size = 10)
throttle_publisher = rospy.Publisher("/motor/duty_cycle", Float64, queue_size = 10)

def steering_callback(data):
    global steering
    steering.data = (data.data + 100.0)/200.0

def throttle_callback(data):
    global throttle
    throttle = data

if __name__ == '__main__':
    global steering
    global throttle
    steering = Float64()
    throttle = Float64()
    rospy.init_node('basic_racecar_control_node')
    rospy.Subscriber("steering_control", Float64, steering_callback)
    rospy.Subscriber("throttle_control", Float64, throttle_callback)
    steering_publisher.publish(steering)
    throttle_publisher.publish(throttle)
    rospy.spin()
