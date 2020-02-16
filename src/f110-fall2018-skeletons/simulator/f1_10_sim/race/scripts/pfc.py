#!/usr/bin/env python
"""
File:   potential_field_controller.py
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

from race.msg import drive_param # For simulator
#from racecar.msg import drive_param # For actual car


class PotentialFieldController(object):
    def __init__(self, car_width=0.5, scan_width=270.0, lidar_range=10.0, turn_clearance=0.35, max_turn_angle=34.0,
                 min_speed=0.1, max_speed=1, min_dist=0.1, max_dist=3.0, no_obst_dist=5.0):
        """
        Todo: explanation of what is

        :param car_width:       (float) Half the car's width with tolerance used for calculating todo Default=0.5
        :param scan_width:      (float) The arc width of the full Lidar scan in degrees.
                                            Default=270.0 for Hokuyo UST-10LX
        :param lidar_range:     (float) Maximum range of the Lidar in meters. Default=10.0 for Hokuyo UST-10LX
        :param turn_clearance:  (float) This is the radius, in meters, to the left or right of the car that must be
                                            clear when the car is attempting to turn left or right. Default=0.35
        :param max_turn_angle:  (float) Maximum steering angle of the car, in degrees. Default=34.0
        :param min_speed:       (float) Slowest speed the car can go in m/s. Made negative for backing up. Default=0.5
        :param max_speed:       (float) Maximum speed the car can go in m/s. Default=2.77
        :param min_dist:        (float) The closest the car should be allowed to get to a wall in meters. Default=0.15
        :param max_dist:        (float) TODO
        :param no_obst_dist:    (float) The furthest distance, in meters, the car will care about obstacles in its way.
                                            Default=6.0
        """

        # Easily changeable parameters for tuning
        self.force_scale_x = 0.3
        self.force_scale_y = 0.1
        self.force_offset_x = 100.0
        self.force_offset_y = 20.0
        self.steering_p_gain = 1.0
        self.steering_d_gain = 0.1

        # Save input parameters
        self.car_width = car_width
        self.scan_width = scan_width
        self.lidar_range = lidar_range
        self.turn_clearance = turn_clearance
        self.max_turn_angle = max_turn_angle
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.min_distance = min_dist
        self.max_distance = max_dist
        self.no_obstacles_distance = no_obst_dist

        # Initialize publisher for speed and angle commands
        self.pub_drive_param = rospy.Publisher('racecar/drive_parameters', drive_param, queue_size=5)

        # Initialize subscriber for the Lidar
        rospy.Subscriber('racecar/scan', LaserScan, self.lidar_callback)

        # Initialize angle forces in order to use a PD controller
        self.last_angle_force = 0.0

    def clip_ranges(self, ranges, min_angle, max_angle, angle_step, measured_max, measured_min):
        """
        TODO: write and explain what this do
        :param ranges:
        :param min_angle:
        :param max_angle:
        :param angle_step:
        :param measured_max:
        :param measured_min:
        :return:
        """

        # Create an array of the measured angles
        angles = np.arange(min_angle, max_angle, angle_step, dtype=np.float32)

        # Compute the indices of values within +/- 90 degrees
        ninety_degrees = math.pi / 2
        indices = np.where(((-1 * ninety_degrees) <= angles) & (angles <= ninety_degrees))

        # Only use values within +/- 90 degrees
        clipped_ranges = ranges[indices]
        clipped_angles = angles[indices]

        #TODO: Other clipping and data manipulation to enforce bounds?

        # Ignore ranges further than the no_obstacle_distance
        indices = np.where(clipped_ranges >= self.no_obstacles_distance)
        clipped_ranges[indices] = measured_max

        # Make measures collected within the minimum distance more important
        indices = np.where(clipped_ranges <= self.min_distance)
        clipped_ranges[indices] = measured_min

        return clipped_ranges, clipped_angles

    def compute_forces(self, ranges, angles):
        """
        TODO: write and explain what this do
        :param ranges:
        :param angles:
        :return:
        """
        # Compute the indices of useful ranges for determining the x forces (+/- 30 degrees)
        thirty_degrees = math.pi / 6
        x_indices = np.where(((-1 * thirty_degrees) <= angles) & (angles <= thirty_degrees))

        # Compute useful values using the data
        neg_ranges_2 = -1 * np.square(ranges)
        angles_cos = np.cos(angles[x_indices])
        angles_sin = np.sin(angles)

        # Compute the forces that are larger the closer to the car they are
        forces_x = np.divide(angles_cos, neg_ranges_2[x_indices])
        forces_y = np.divide(angles_sin, neg_ranges_2)

        # Compute the net forces, including the offsets
        net_force_x = self.force_scale_x * float(np.sum(forces_x)) + self.force_offset_x
        net_force_y = self.force_scale_y * float(np.sum(forces_y)) + self.force_offset_y

        # Compute the angle force and speed force
        angle_force = np.arctan2(net_force_y, self.force_offset_x) # net_force_x)
        speed_force = max((net_force_x*abs(net_force_x) - abs(net_force_y)), 0) # Note: y forces are subtractive. If the car is close to a wall or turning, we want to go slower
        #prevent division by zero
        if(not (abs(speed_force)>0)):
            return
        speed_force = (speed_force / abs(speed_force)) * math.sqrt(abs(speed_force))

        return speed_force, angle_force

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
        measured_max = data.range_max
        measured_min = data.range_min
        clipped_ranges, clipped_angles = self.clip_ranges(ranges, min_angle, max_angle, angle_step, measured_max,
                                                          measured_min)

        # Compute forces from the potential field acquired from the Lidar scan
        speed_force, angle_force = self.compute_forces(clipped_ranges, clipped_angles)

        # Compute speed and steering commands, then publish them
        self.publish_commands(speed_force, angle_force)

        return

    def publish_commands(self, speed_force, angle_force):
        """
        TODO: describe what do
        :param speed_force: (float)
        :param angle_force: (float)
        """

        # Convert angle_force to angle command
        angle_cmd = self.steering_p_gain * angle_force + self.steering_d_gain * (angle_force - self.last_angle_force)
        self.last_angle_force = angle_force

        # Make sure angle command is within steering settings
        if abs(angle_cmd) > self.max_turn_angle:
            angle_cmd = (angle_cmd / abs(angle_cmd)) * self.max_turn_angle

        # Convert speed_force to speed command
        if speed_force <= 0: # In the case that the car needs to backup, flip the turning angle
            angle_cmd = -1 * angle_cmd
            speed_cmd = -1 * self.min_speed
        else:
            speed_percentage = speed_force / self.force_offset_x
            speed_cmd = speed_percentage * (self.max_speed - self.min_speed) + self.min_speed

        # Publish the command
        msg = drive_param()
        msg.angle = angle_cmd
        msg.velocity = speed_cmd
        self.pub_drive_param.publish(msg)
        rospy.loginfo('speed [force, cmd]: [' + str(speed_force) + ', ' + str(speed_cmd) + '] angle [force, cmd]: [' + str(angle_force) + ', ' + str(angle_cmd) + ']')
        return


if __name__ == '__main__':
    rospy.init_node('potential_field_control', anonymous=True)
    extendObj = PotentialFieldController()
    rospy.Subscriber('scan', LaserScan, extendObj.lidar_callback)
    rospy.spin()
