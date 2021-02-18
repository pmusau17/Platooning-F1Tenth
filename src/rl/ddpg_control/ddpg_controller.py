#!/usr/bin/env python
import time
import gc
import os
import numpy as np
import copy
import torch
import torch.nn as nn
import torch.nn.functional as F
from numpy.core.multiarray import ndarray
from sensor_msgs.msg import LaserScan
from race.msg import angle_msg
import rospy
import copy
import math
import rospkg 

from race.msg import drive_param # For simulator
#from racecar.msg import drive_param # For actual car

class ActorNN(nn.Module):
    def __init__(self, num_inputs=4, hidden_size1=64, hidden_size2=64, num_actions=2, final_bias=3e-3):
        super(ActorNN, self).__init__()
        """
        This Neural Network architecture creates a full actor, which provides the control output. The architecture is 
        derived from the original DDPG paper.

        :param num_inputs:  (int)   The desired size of the input layer. Should be the same size as the number of 
                                        inputs to the NN. Default=4
        :param hidden_size1:(int)   The desired size of the first hidden layer. Default=400
        :param hidden_size2:(int)   The desired size of the second hidden layer. Default=300
        :param num_actions: (int)   The desired size of the output layer. Should be the same size as the number of 
                                        outputs from the NN. Default=2
        :param final_bias:  (float) The final layers' weight and bias range for uniform distribution. Default=3e-3
        """

        # The first layer
        self.linear1 = nn.Linear(num_inputs, hidden_size1)

        # The second layer
        self.linear2 = nn.Linear(hidden_size1, hidden_size2)

        # The output layer
        self.out = nn.Linear(hidden_size2, num_actions)

    def forward(self, state):
        """
        This function performs a forward pass through the network.

        :param state: (tensor)   The input state the NN uses to compute an output.
        :return mu:   (tensor)   The output of the NN, which is the action to be taken.
        """

        # Pass through layer 1
        x = self.linear1(state)
        x = F.relu(x)

        # Pass through layer 2
        x = self.linear2(x)
        x = F.relu(x)

        # Pass through the output layer
        x = self.out(x)

        # Output is scaled using tanh
        mu = torch.tanh(x)

        # Return the result
        return mu

class DDPGController(object):
    def __init__(self, policy_file_name, lidar_sub_name, control_pub_name, rate=10, mode=0,
                 min_turn_angle=(-34.0 * np.pi / 180.),
                 max_turn_angle=(34.0 * np.pi / 180.),
                 constant_velocity=1.0, max_lidar_range=10.0, decoupled=True):
        """
        TODO explain the inputs
        """

        # Save the input values
        self.rate = rospy.Rate(rate)
        self.mode = mode
        self.min_turn_angle = min_turn_angle
        self.max_turn_angle = max_turn_angle
        self.constant_velocity = constant_velocity
        self.max_lidar_range = max_lidar_range
        self.decoupled = decoupled

        # Initialize the control policy
        self.target_angles = np.asarray([-90., -60., -45., -30, 0., 30., 45., 60., 90.]) * (np.pi / 180.)
        self.control_nn = ActorNN(num_inputs=len(self.target_angles), hidden_size1=64, hidden_size2=64,
                                  num_actions=1, final_bias=3e-3)  #.to(self.device)

        # Load the control policy
        if policy_file_name is not None:
            self.__load_model(policy_file_name)

        # Initialize useful variables
        self.indices = None
        self.lidar_ranges = None
        self.scale_mult = (self.max_turn_angle - self.min_turn_angle) / 2.0
        self.scale_add = (self.max_turn_angle - self.min_turn_angle) / 2.0 + self.min_turn_angle

        # Initialize publisher for speed and angle commands
        if(self.decoupled):
            self.pub_drive_param = rospy.Publisher(control_pub_name, angle_msg, queue_size=5)
        else:
            self.pub_drive_param = rospy.Publisher(control_pub_name, drive_param, queue_size=5)


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

            # Compute the indices of the desired lidar angles
            self.indices = range(len(self.target_angles))
            for i in range(len(self.target_angles)):
                self.indices[i] = int(round((self.target_angles[i] - min_angle) / angle_step))
            # print(angles[self.indices])

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
        # Forward pass the network
        state = torch.FloatTensor(obs)  #.to(self.device)
        action = self.control_nn.forward(state)
        # action = action.cpu()

        # Convert to numpy array
        np_action = action.detach().numpy()
        
        # Clip the output so that it is within the steering limits
        steering_angle = np.multiply(np_action, self.scale_mult) + self.scale_add 

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

            # Store the saved models
            self.control_nn.load_state_dict(checkpoint['actor'])

            # Evaluate the neural networks to ensure the weights were properly loaded
            self.control_nn.eval()

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
        if(self.decoupled):
            msg=angle_msg()
            msg.header.stamp=rospy.Time.now()
            msg.steering_angle=steering_angle
        else:
            msg = drive_param()
            msg.header.stamp = rospy.Time.now()
            msg.angle = steering_angle
            msg.velocity = velocity
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
    rospy.init_node('ddpg_controller', anonymous=True)
    package_path=rospkg.RosPack().get_path('rl')
    load_name = os.path.join(package_path,"ddpg_control","final_models","final_model.pth")
    
    mode = 1
    vel = 1.0

    #get the arguments passed from the launch file
    args = rospy.myargv()[1:]
    #get the racecar name so we know what to subscribe to
    racecar_name=args[0]
    lidar_sub_name = racecar_name+'/scan'
    #if there's more than two arguments then its decoupled
    if len(args)>1:
        control_pub_name= racecar_name+'/angle_msg'
        decouple = True
    else:
        control_pub_name = 'racecar/drive_parameters'
        decouple = False
        
    
    extendObj = DDPGController(policy_file_name=load_name, lidar_sub_name=lidar_sub_name, 
                              control_pub_name=control_pub_name, rate=20, mode=mode,
                              min_turn_angle=(-34.0 * np.pi / 180.),
                              max_turn_angle=(34.0 * np.pi / 180.),
                              constant_velocity=vel, max_lidar_range=10.0,decoupled=decouple)
    
    if mode == 0:

        rospy.spin()
    else:
        #
        extendObj.run()