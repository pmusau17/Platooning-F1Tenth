#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0

def set_throttle_steer(data):

    global flag_move
    global racecar_name

    pub_vel_left_rear_wheel = rospy.Publisher('/'+racecar_name+'/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/'+racecar_name+'/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/'+racecar_name+'/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/'+racecar_name+'/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/'+racecar_name+'/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/'+racecar_name+'/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    # Velocity is in terms of radians per second.
    # Want to go 1 m/s with a wheel of radius 0.05m. This translates to 19.97 radians per second, roughly 20.
    # However, at a multiplication factor of 20 speed is half of what it should be, so doubled to 40.
    throttle = data.drive.speed * 40.0
    steer = data.drive.steering_angle

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)

def servo_commands():
    global racecar_name
    rospy.init_node('servo_commands', anonymous=True)

    rospy.Subscriber("/"+racecar_name+"/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        args = rospy.myargv()[1:]
        racecar_name=args[0]
        servo_commands()
    except rospy.ROSInterruptException:
        pass
