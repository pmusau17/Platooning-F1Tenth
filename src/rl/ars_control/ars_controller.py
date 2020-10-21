#!/usr/bin/env python
import time
import gc
import os
import numpy as np
import copy
import torch
from numpy.core.multiarray import ndarray
from sensor_msgs.msg import LaserScan
from race.msg import angle_msg
import rospy
import copy
import math
import rospkg 

from race.msg import drive_param # For simulator
#from racecar.msg import drive_param # For actual car

class ARSController(object):
    def __init__(self, policy_file_name, lidar_sub_name, control_pub_name, rate=10, mode=0,
                 min_turn_angle=(-34.0 * np.pi / 180.),
                 max_turn_angle=(34.0 * np.pi / 180.),
                 constant_velocity=1.0,
                 lidar_angle=(np.pi / 2), max_lidar_range=10.0):
        """
        TODO explain the inputs
        """

        # Save the input values
        self.rate = rospy.Rate(rate)
        self.mode = mode
        self.min_turn_angle = min_turn_angle
        self.max_turn_angle = max_turn_angle
        self.constant_velocity = constant_velocity
        self.lidar_angle = lidar_angle
        self.max_lidar_range = max_lidar_range

        # Load the control policy
        if policy_file_name is not None:
            self.__load_model(policy_file_name)

        # Initialize useful variables
        self.indices = None
        self.lidar_ranges = None

        # Initialize publisher for speed and angle commands
        self.pub_drive_param = rospy.Publisher(control_pub_name, angle_msg, queue_size=5)

        # Initialize subscriber for the Lidar
        rospy.Subscriber(lidar_sub_name, LaserScan, self.__callback_lidar)
    
    def __callback_lidar(self, data):
        """
        This callback method responds when a lidar scan is received. The lidar data is clipped to eliminate extraneous
        maximums. The result is saved as the state.

        :param data:    (sensor_msgs.LaserScan) The lidar data message
        """
        # Collect the ranges
        ranges = np.asarray(data.ranges)

        # If the indices of ranges hasn't been determined yet, determine them
        if self.indices is None:
            min_angle = data.angle_min
            max_angle = data.angle_max
            angle_step = data.angle_increment

            # Create an array of the measured angles
            angles = np.arange(min_angle, max_angle, angle_step, dtype=np.float32)

            # Compute the indices of values within +/- the desired lidar angle
            self.indices = np.where(((-1 * self.lidar_angle) <= angles) & (angles <= self.lidar_angle))
            # print(len(self.indices[0]))

        # Clip any range values larger than the set maximum since the lidar data is very noisy at larger distances
        max_ranges = self.max_lidar_range * np.ones_like(ranges)
        clipped_ranges = np.minimum(ranges, max_ranges)
        clipped_ranges = clipped_ranges[self.indices]

        if self.mode == 0:
            # If the mode indicates using the callback for setting the publish rate, determine the control output 
            # and publish it
            action = self.__get_control(clipped_ranges)
            self.__publish_cmd(self.constant_velocity, action)
        else:
            # Otherwise, there is a set rate for publishing the control action. Instead, save the observation info for 
            # when the control output needs to be determined
            self.lidar_ranges = copy.copy(clipped_ranges)

        return
    
    def __get_control(self, obs):
        """
        This method determines the desired control value from a given observation.

        :param obs:             (np.array)         The input observation as an array
        :return steering_angle: (np.array)         The desired control value
        """

        action = self.policy_theta.dot(obs.T)

        # # Return the index of the maximum value from the action if the action space is discrete
        # if self.is_discrete:
        #     action = np.argmax(action)
        
        # Clip the output so that it is within the steering limits
        steering_angle = max(min(action, self.max_turn_angle), self.min_turn_angle)

        return steering_angle
    
    def __load_model(self, load_path):
        """
        This method loads a model. The loaded model can be a previously learned policy or an initializing policy used
        for consistency.

        :input:
            :param load_path: (string) The file name where the models will be loaded from. default=None
        """

        # Load the saved file as a dictionary
        if load_path is not None:
            checkpoint = torch.load(load_path)

            self.policy_theta = checkpoint['theta']

        # Clean up any garbage that's accrued
        gc.collect()

        return

    def __publish_cmd(self, velocity, steering_angle):
        """
        TODO: documentation
        :param velocity:
        :param steering_angle:
        :return:
        """
        msg=angle_msg()
        msg.header.stamp=rospy.Time.now()
        msg.steering_angle=steering_angle
        # msg = drive_param()
        # msg.angle = steering_angle
        # msg.velocity = velocity
        self.pub_drive_param.publish(msg)

        return

    def run(self):
        """
        TODO write this function and explain it
        """

        for _ in range(20):
            self.__publish_cmd(0.0, 0.0)
            self.rate.sleep()

        while not rospy.is_shutdown():
            action = self.__get_control(self.lidar_ranges)
            self.__publish_cmd(self.constant_velocity, action)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('ars_controller', anonymous=True)
    package_path=rospkg.RosPack().get_path('rl')
    load_name = os.path.join(package_path,"ars_control","final_models","step_7062_model.pth")
    lidar_sub_name = 'racecar/scan'
    
    #control_pub_name = 'racecar/drive_parameters'
    control_pub_name= 'racecar/angle_msg'
    mode = 1
    vel = 1.0
    extendObj = ARSController(policy_file_name=load_name, lidar_sub_name=lidar_sub_name, 
                              control_pub_name=control_pub_name, rate=20, mode=mode,
                              min_turn_angle=(-34.0 * np.pi / 180.),
                              max_turn_angle=(34.0 * np.pi / 180.),
                              constant_velocity=vel,
                              lidar_angle=(np.pi / 2), max_lidar_range=10.0)
    
    if mode == 0:

        rospy.spin()
    else:
        #
        extendObj.run()