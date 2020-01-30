#!/usr/bin/env python
"""
File:   wall_follower.py
Author: Nathaniel Hamilton

Description: blah

Usage:  blah

Remaining Tasks:
        * Writing everything
"""
from numpy.core.multiarray import ndarray
from sensor_msgs.msg import LaserScan
import time
import rospy
import copy
import numpy as np
import math

from race.msg import drive_param  # For simulator

# from racecar.msg import drive_param # For actual car

# Define some constants
LEFT = 0
BOTH = 1
RIGHT = 2


class WallFollowingControl(object):
    def __init__(self, control_pub_name, car_width=0.5, lidar_range=10.0, max_turn_angle=34.0,
                 min_speed=0.1, max_speed=3.0, target_dist=0.2, which_wall='left'):
        """

        :param control_pub_name:
        :param car_width:
        :param lidar_range:
        :param max_turn_angle:
        :param min_speed:
        :param max_speed:
        :param target_dist:
        """
        # Save input parameters
        self.car_width = car_width
        self.lidar_range = lidar_range
        self.max_turn_angle = max_turn_angle * math.pi / 180.0
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.target_distance = target_dist

        if which_wall == 'left':
            self.wall = LEFT
        elif which_wall == 'right':
            self.wall = RIGHT
        else:
            self.wall = BOTH

        # Initialize publisher for speed and angle commands
        self.pub_drive_param = rospy.Publisher(control_pub_name, drive_param, queue_size=5)

        # Initialize PD control variables
        self.error_constant = 1.0
        self.last_angle = 0.0
        self.steering_p_gain = 1.0
        self.steering_d_gain = 0.1

    def compute_command(self, distance_to_wall, which_side):
        """

        :param distance_to_wall:
        :param which_side:
        :return:
        """

        # Compute the angle between the current distance and the desired distance from the wall
        error = distance_to_wall - self.target_distance
        angle = np.arctan2(error, self.error_constant)

        # Flip the result depending on the wall
        if which_side == RIGHT:
            angle *= -1.0

        # Apply a PD control to the desired angle
        desired_angle = self.steering_p_gain * angle + self.steering_d_gain * (angle - self.last_angle)
        self.last_angle = angle

        # TODO: make the speed variable?
        desired_speed = self.max_speed

        return desired_speed, desired_angle

    def compute_wall_distances(self, ranges, min_angle, max_angle, angle_step):
        """
        TODO: write and explain what this do
        :param ranges:
        :param min_angle:
        :param max_angle:
        :param angle_step:
        :return:
        """

        # Create an array of the measured angles
        angles = np.arange(min_angle, max_angle, angle_step, dtype=np.float32)

        # Compute the indeces for angles to look at on the right (negative)
        ninety_degrees = math.pi / 2.0
        fifteen_degrees = math.pi / 12.0
        right_indeces = np.where(((-1 * (ninety_degrees + fifteen_degrees)) <= angles) &
                                 ((-1 * (ninety_degrees - fifteen_degrees)) >= angles))

        # Compute the indeces for angles to look at on the left (positive)
        left_indeces = np.where(((ninety_degrees + fifteen_degrees) >= angles) &
                                ((ninety_degrees - fifteen_degrees) <= angles))

        # Compute distance to right wall
        right_distances = ranges[right_indeces] * np.sin(angles[right_indeces])
        right_wall_distance = np.absolute(np.mean(right_distances))

        # Compute distance to left wall
        left_distances = ranges[left_indeces] * np.sin(angles[left_indeces])
        left_wall_distance = np.absolute(np.mean(left_distances))

        # Which is closer?
        if left_wall_distance <= right_wall_distance:
            which_side_is_closer = LEFT
        else:
            which_side_is_closer = RIGHT

        return left_wall_distance, right_wall_distance, which_side_is_closer

    def lidar_callback(self, data):
        """
        TODO: write and explain what this do
        :param data:
        :return:
        """
        # Reduce the Lidar data to only scans between +/- 90 degrees
        ranges = np.asarray(data.ranges)
        min_angle = data.angle_min
        max_angle = data.angle_max
        angle_step = data.angle_increment
        left_wall_dist, right_wall_dist, left_or_right = self.compute_wall_distances(ranges, min_angle, max_angle,
                                                                                     angle_step)

        # Determine which wall to follow
        if self.wall == LEFT:
            wall_dist = left_wall_dist
            side = LEFT
        elif self.wall == RIGHT:
            wall_dist = right_wall_dist
            side = RIGHT
        else:
            if left_or_right == LEFT:
                wall_dist = left_wall_dist
                side = LEFT
            else:
                wall_dist = right_wall_dist
                side = RIGHT

        # Compute forces from the potential field acquired from the Lidar scan
        speed, angle = self.compute_command(wall_dist, side)

        # Compute speed and steering commands, then publish them
        self.publish_commands(speed, angle)

        return

    def publish_commands(self, speed, angle):
        """

        :param speed:   (float)
        :param angle:   (float)
        :return:
        """

        # Make sure angle command is within steering settings
        angle_cmd = max(min(angle, self.max_turn_angle), -self.max_turn_angle)

        # TODO: Add something to handle backwards motion
        # Make sure the desired speed is within the set bounds
        speed_cmd = max(min(speed, self.max_speed), self.min_speed)

        # Publish the command
        msg = drive_param()
        msg.angle = angle_cmd
        msg.velocity = speed_cmd
        self.pub_drive_param.publish(msg)
        rospy.loginfo('speed: ' + str(speed_cmd) + ' angle: ' + str(angle_cmd))
        return


if __name__ == '__main__':
    rospy.init_node('wall_follow_control', anonymous=True)
    extendObj = WallFollowingControl(control_pub_name='racecar/drive_params',
                                     car_width=0.5,
                                     lidar_range=10.0,
                                     max_turn_angle=34.0,
                                     min_speed=0.1,
                                     max_speed=3.0,
                                     target_dist=0.3,
                                     which_wall='left')
    rospy.Subscriber('scan', LaserScan, extendObj.lidar_callback)
    rospy.spin()

    # TEST CODE
    # ranges = np.arange(0, 270, 1)
    # min_angle = -135. / 180 * math.pi
    # max_angle = 135. / 180 * math.pi
    # angle_step = 1. / 180 * math.pi
    # left_dist, right_dist, side = extendObj.compute_wall_distances(ranges, min_angle, max_angle, angle_step)
    # print(left_dist)
    # print(right_dist)
    # print(side)
