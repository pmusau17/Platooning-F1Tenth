#!/usr/bin/env python
"""
File:   ppo.py
Author: Nathaniel Hamilton

Description: blah
    State is an array with the following information:
        [forward distance to lead vehicle, sideways distance to lead vehicle, speed_lead - speed_ego, yaw_lead - yaw_ego]
    Action is an array with the following information:
        [acceleration, steering adjustment]

Usage:  blah

Class Functions:
    * calculate_reward(ndarray, ndarray) (not done)
    * get_action_and_value(ndarray)
    * get_state()  (not done)
    * learn(int, int, int, int) (kinda done)
    * publish_command(float, float)
    * reset (not done)
    * step(int)
    * test(int)
    * update(list, list, list, list, list)

Published Topics:

Subscribed Topics:

Remaining Tasks:
    * Writing everything
    * add logging points
"""
from numpy.core.multiarray import ndarray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import time
import rospy
import math
import os
import numpy as np
import torch
import torch.optim as optim
from torch.autograd import Variable
from tf.transformations import euler_from_quaternion

from class_nn import *

from race.msg import drive_param # For simulator
# from racecar.msg import drive_param # For actual car


def calculate_entropy(std_devs):
    """
    Entropy of a multivariate Gaussian is calculated as 0.5ln(det(2*pi*e*Var)). Given that the covariance matrix of the
    action is diagonal and 2x2, this equation can be simplified to 0.5*(ln(2*pi*e)+ln(var1)+ln(var2)). Simplifying
    further to utilize the standard deviations used as input, the equation becomes:
        entropy = 0.5*ln(2*pi*e) + ln(std) + ln(std2)

    :param std_devs:    (FloatTensor)

    :return entropy:    (float)
    """

    # Compute the natural log of the standard deviations
    logstds = torch.log(std_devs)

    # Compute the constant
    c = 0.5 * np.log((2 * np.pi * np.e))

    entropy = c + logstds.sum().numpy()

    return entropy


def calculate_log_probability(x, mu, std_devs):
    """
    The log likelihood of a multivariate Gaussian is computed using the following formula:
        ln(L) = -0.5(ln(det(Var) + (x-mu)'*Var^(-1)*(x-mu) + kln(2*pi))
        ln(L) = -0.5(2*sum(ln(std_i)) + sum(((x_i - mu_i)/std_i)^2) + ln(2*pi))

    :param x:           (ndarray)
    :param mu:          (FloatTensor)
    :param std_devs:    (FloatTensor)

    :return log_prob:   (float)
    """

    # Compute first term
    logstds = torch.log(std_devs)
    a = 2 * torch.sum(logstds).numpy()

    # Compute second term
    x = torch.FloatTensor(x)
    b = torch.sum(torch.square((x - mu) / std_devs)).numpy()

    # Compute third term
    c = np.log(2*np.pi)

    # Combine the terms
    log_prob = -0.5 * (a + b + c)

    return log_prob


def compute_returns(next_value, rewards, values, gamma, lam):
        """
        This function computes the returns after an episode has been completed. returns are the estimated state-action
        value, i.e. Q-value

        :input:
            :param next_value:  (float)     The estimated value of the next state from the critic.
            :param rewards:     (list)      The rewards collected during the episode.
            :param values:      (list)      The estimated values during the episode from the critic.
            :param gamma:       (float)
            :param lam:         (float)
        :output:
            :return returns:    (ndarray)   The computed returns for the episode.
        """

        """
        Implementation without GAE. Old.
        return_ = next_value
        returns = np.zeros_like(values)
        for t in reversed(range(len(rewards))):
            return_ = rewards[t] + gamma * return_
            returns[t] = return_

        return returns
        """

        future_val = next_value
        gae = 0
        returns = np.zeros_like(values)
        for t in reversed(range(len(rewards))):
            delta = rewards[t] + gamma * future_val - values[t]
            gae = delta + gamma * lam * gae
            future_val = values[t]
            returns[t] = gae

        return returns


def reset_env():
    """

    """
    #TODO: write this
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    reset_world()
    return


class PPO(object):
    def __init__(self, control_pub_name, car_width=0.5, scan_width=270.0, lidar_range=10.0, turn_clearance=0.35,
                 max_turn_angle=34.0, min_speed=0.5, max_speed=4.0, min_dist=0.1, max_dist=3.0, no_obst_dist=10.0,
                 crash_threshold=10, env='sim', rate=20, load_path=None, save_interval=10, save_path='.',
                 episode_length=8192, gamma=0.99, lam=0.95, learning_rate=2e-4, rl_clip_range=0.1):
        """

        """

        # Save the input parameters
        self.car_width = car_width
        self.scan_width = scan_width
        self.lidar_range = lidar_range
        self.turn_clearance = turn_clearance
        self.max_turn_angle = max_turn_angle
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.min_dist = min_dist
        self. max_dist = max_dist
        self.no_obst_dist = no_obst_dist
        self.crash_threshold = crash_threshold
        self.env = env
        self.rate = rospy.Rate(rate)  # TODO: Look into this. might want to be a constant wait time
        self.load_path = load_path
        self.save_intv = save_interval
        self.save_path = save_path
        self.episode_length = episode_length
        self.gamma = gamma
        self.lam = lam
        self.lr = learning_rate
        self.clip_range = rl_clip_range

        # Initialize publishers
        self.pub_drive_param = rospy.Publisher(control_pub_name, drive_param, queue_size=5)

        # Initialize Cuda variables
        use_cuda = torch.cuda.is_available()
        self.device = torch.device("cuda" if use_cuda else "cpu")

        # Create the actor and critic neural network
        self.ac_nn = ActorCriticNN().to(self.device)

        # Create the optimizers for the actor and critic neural networks
        self.ac_optimizer = optim.Adam(self.ac_nn.parameters(), lr=self.lr)

        # Initialize the NN and optimizer
        if load_path is None or load_path == 'None':
            self.ac_nn.initialize_orthogonal()
        else:
            self.load_models(self.load_path)

        # Initialize global variables used by subscribers
        self.ego_pos = None  # (x, y, yaw, speed)
        self.lead_pos = None  # (x, y, yaw, speed)
        self.lidar_done = 0

    def calculate_reward(self, curr_state):
        """

        :param curr_state:  (ndarray)   Current state of the agent
        :return reward:     (float)     The reward associated with the state information.
        """

        goal_state = np.array([0.5, 0.0, 0.0, 0.0])
        mask = np.ndarray([1, 1, 1, 1])

        diff = np.absolute(goal_state - curr_state)
        diff = np.multiply(diff, mask)
        reward = -np.sum(diff)

        return reward

    def callback_ego_odom(self, data):
        """

        """
        qx = data.orientation.x
        qy = data.orientation.y
        qz = data.orientation.z
        qw = data.orientation.w

        quaternion = (qx, qy, qz, qw)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]

        x = data.position.x
        y = data.position.y

        self.ego_pos = [x, y, yaw]

    def callback_leader_odom(self, data):
        """

        """
        x = data.position.x
        y = data.position.y

        self.lead_pos = [x, y]

    def callback_lidar(self, data):
        """
        TODO: write and explain what this do
        :param data:
        """
        # Pull the useful information from the message
        ranges = np.asarray(data.ranges)
        min_angle = data.angle_min
        max_angle = data.angle_max
        angle_step = data.angle_increment

        # Create an array of the measured angles
        angles = np.arange(min_angle, max_angle, angle_step, dtype=np.float32)

        # Compute the indices of values within +/- 90 degrees
        ninety_degrees = math.pi / 2
        indices = np.where(((-1 * ninety_degrees) <= angles) & (angles <= ninety_degrees))

        # Only use values within +/- 90 degrees
        clipped_ranges = ranges[indices]
        clipped_angles = angles[indices]

        # Find readings that are below the set minimum. If there are multiple readings below the threshold, a crash
        # likely occurred and the episode should end
        indices = np.where(clipped_ranges <= self.min_dist)
        if len(indices) >= self.crash_threshold:
            self.lidar_done = 1

    def get_action_and_value(self, state):
        """
        This function uses the CNN to determine which action to take next and the estimated value of the current state.
        The action is randomly selected given the output distribution.

        :input:
            :param state:           (ndarray)   Input image to determine which action to take
        :outputs:
            :return distribution:   (np.array)  The action distribution used to determine the next action
            :return action:         (ndarray)     The chosen action to take
        """

        # Forward pass the network
        state = torch.FloatTensor(state).to(self.device)
        means, stds, val = self.ac_nn.forward(state)
        means = means.cpu()
        stds = stds.cpu()
        val = val.cpu()

        # Choose a random action according to the distribution
        random_val = torch.FloatTensor(np.random.rand(2)).cpu()
        action = (means + stds * random_val).numpy()

        # Calculate entropy
        entropy = calculate_entropy(stds)

        # Convert value to numpy compatible version
        value = val.detach().numpy()[0, 0]

        return action, means, stds, entropy, value

    def get_state(self):
        """

        :return state:  (ndarray)
        :return done:   (Boolean)
        """
        lidar_done = self.lidar_done

        # The state information is collected differently in simulation from the real world
        if self.env == 'sim':
            # Collect and save the latest information for computing the state
            ego_pos = self.ego_pos
            lead_pos = self.lead_pos

            # Break up the position information for easier understanding
            ego_x = ego_pos[0]
            ego_y = ego_pos[1]
            ego_yaw = ego_pos[2]
            ego_speed = ego_pos[3]
            lead_x = lead_pos[0]
            lead_y = lead_pos[1]
            lead_yaw = lead_pos[2]
            lead_speed = lead_pos[3]

            # Compute the state information
            # [forward distance to lead vehicle, sideways distance to lead vehicle, speed_lead - speed_ego,
            #  yaw_lead - yaw_ego]
            dx = lead_x - ego_x
            dy = lead_y - ego_y
            forward_dist = dx * np.cos(ego_yaw) + dy * np.sin(ego_yaw)
            side_dist = dy * np.cos(ego_yaw) - dx * np.sin(ego_yaw)
            speed_dif = lead_speed - ego_speed
            yaw_dif = lead_yaw - ego_yaw

            # Compile the state info into one array
            state = np.asarray([forward_dist, side_dist, speed_dif, yaw_dif], float)

        else:
            # TODO: figure out how to do this
            # Collect and save the latest information for computing the state

            # Break up the information for easier understanding

            # TODO: where from??
            forward_dist = 0.0
            side_dist = 0.0
            speed_dif = 0.0
            yaw_dif = 0.0

            # Compile the state info into one array
            state = np.asarray([forward_dist, side_dist, speed_dif, yaw_dif], float)

        # Determine if the state should be terminal
        done = 0
        if (forward_dist > self.max_dist) or (forward_dist < self.min_dist) or (lidar_done == 1):
            done = 1

        return state, done

    def learn(self, total_steps=int(1e9), horizon_length=8192, num_epochs=4, minibatch_length=8192):
        """

        """

        # Make sure the Neural Net is in train mode
        self.ac_nn.train()

        # Iterate through a sufficient number of steps broken into horizons
        step_count = 0
        save_count = 0
        while step_count < total_steps:
            h_length = min(horizon_length, (total_steps - step_count))
            # Execute the horizon
            h_states, h_actions, h_log_probs, h_returns, h_values = self.play_horizon(h_length)
            step_count += h_length

            # Iterate through the number of epochs, running updates on shuffled minibatches
            indices = np.arange(h_length)
            for e in range(num_epochs):
                # Shuffle the frames in the horizon
                np.random.shuffle(indices)

                # Update each shuffled minibatch (mb)
                for mb_start in range(0, h_length, minibatch_length):

                    # Single out each minibatch from the recorded horizon
                    mb_end = min((mb_start + minibatch_length), h_length)
                    mb_indices = indices[mb_start:mb_end]
                    mb_states = h_states[mb_indices]
                    mb_actions = h_actions[mb_indices]
                    mb_log_probs = h_log_probs[mb_indices]
                    mb_returns = h_returns[mb_indices]
                    mb_values = h_values[mb_indices]

                    # print('updating...')
                    actor_loss, critic_loss = self.update(mb_states, mb_actions, mb_log_probs, mb_returns, mb_values)

            # Save the model at the desired rate
            if ((save_count % self.save_intv == 0) or (step_count >= total_steps)) and (self.save_path is not None):
                # Create the save path name and save it
                print('saving...')
                save_path = self.save_path + 'step_' + str(step_count) + '_model.pth'
                self.save_models(save_path=save_path)
            save_count += 1

        return

    def load_models(self, load_path=None):
        """
        This function loads the neural network models and optimizers for both the actor and the critic from one file.
        If a load_path is not provided, the function will not execute. For more examples on how to save/load models,
        visit https://pytorch.org/tutorials/beginner/saving_loading_models.html

        :param load_path:  (string) The file name where the models will be loaded from. default=None
        """

        # Load the saved file as a dictionary
        if load_path is not None:
            checkpoint = torch.load(load_path)

            # Store the saved models
            self.ac_nn.load_state_dict(checkpoint['actor_critic_model'])
            self.ac_optimizer.load_state_dict(checkpoint['ac_optimizer'])

            # Evaluate the neural network to ensure the weights were properly loaded
            self.ac_nn.eval()

        return

    def play_horizon(self, horizon_length):
        """

        """

        # Initialize variables
        steps_taken = 0
        states = []
        actions = []
        log_probs = []
        returns = []
        values = []

        # Make sure the environment starts fresh if in simulation
        if self.env == 'sim':
            reset_env()
            self.lidar_done = 0

        # Play through episodes and record the results until the horizon has been played through
        while steps_taken < horizon_length:
            # Determine how many steps to take in the next episode
            max_steps = min((horizon_length - steps_taken), self.episode_length)

            # Execute an episode (returns if done or reached desired number of steps)
            ep_states, ep_actions, ep_log_probs, ep_returns, ep_values, steps, ep_reward, done = self.test(max_steps)

            # Update the records
            steps_taken += steps
            states.append(ep_states)
            actions.append(ep_actions)
            log_probs.append(ep_log_probs)
            returns.append(ep_returns)
            values.append(ep_values)

            #TODO: do something with the ep_reward...

            # Only reset the environment if a terminal state has been reached
            if done == 1:
                if self.env == 'sim':
                    reset_env()
                    self.lidar_done = 0

        return states, actions, log_probs, returns, values

    def publish_cmd(self, velocity, steering_angle):
        """

        :param velocity:        (float)
        :param steering_angle:  (float)
        """

        angle = max(min(steering_angle, self.max_turn_angle), -self.max_turn_angle)
        vel = max(min(velocity, self.max_speed), self.min_speed)

        msg = drive_param()
        msg.angle = angle
        msg.velocity = vel
        self.pub_drive_param.publish(msg)

        return vel, angle

    def step(self, curr_state, action):
        """

        :param curr_state:  (ndarray)
        :param action:      (ndarray)
        """

        # Publish action
        vel_cmd = curr_state[2] + action[0]
        steer_cmd = curr_state[3] + action[1]
        vel, angle = self.publish_cmd(vel_cmd, steer_cmd)

        # Catch discrepancies between desired and executed actions
        actual_action = action
        actual_action[0] = vel - curr_state[2]
        actual_action[1] = angle - curr_state[3]

        # Wait specified time
        self.rate.sleep()

        # Collect new state
        next_state, done = self.get_state()
        reward = self.calculate_reward(next_state)

        return next_state, actual_action, reward, done

    def save_models(self, save_path='models.pth'):
        """
        This function save the neural network models and optimizers for both the actor and the critic in one file. For
        more examples on how to save/load models, visit
        https://pytorch.org/tutorials/beginner/saving_loading_models.html

        :param save_path:   (string) The file name that the models will be saved to. default='models.pth'
        """

        # Save all aspects of the learner in one file
        torch.save({
            'actor_critic_model':   self.ac_nn.state_dict(),
            'ac_optimizer':         self.ac_optimizer.state_dict()
                    }, save_path)

        return

    def test(self, max_steps=-1):
        """
        This function runs the learned policy until a stop condition occurs.

        :inputs:
            :param max_steps:   (int)   Maximum number of steps to execute in the training cycle. If -1, then there is
                                        no maximum. Default=-1
        """

        # Initialize
        state, done = self.get_state()
        step = 0
        total_reward = 0
        states = []
        actions = []
        log_probs = []
        rewards = []
        values = []
        returns = []

        while not rospy.is_shutdown():
            # Stop the controller if there is a collision or time-out
            if done or (step >= max_steps != -1):
                # stop
                self.publish_cmd(0.0, 0.0)
                break

            # Determine the next action
            action, means, stds, entropy, value = self.get_action_and_value(state)
            log_prob = calculate_log_probability(action, means, stds)

            # Execute determined action
            next_state, actual_action, reward, done = self.step(state, action)

            # Record information about the step
            states.append(state)
            actions.append(actual_action)
            log_probs.append(log_prob)
            rewards.append(reward)
            values.append(value)

            # Update for next step
            total_reward += reward
            state = next_state
            step += 1

        # Compute returns
        _, _, _, _, next_value = self.get_action_and_value(state)
        returns.extend(compute_returns(next_value, rewards, values, self.gamma, self.lam))

        return states, actions, log_probs, returns, values, step, total_reward, done

    def update(self, states, actions, old_log_probs, returns, old_values):
        """
        This function updates neural networks for the actor and critic using back-propogation. More information about
        this process can be found in the PPO paper (https://arxiv.org/abs/1707.06347) and the pytorch implementation
        found at https://github.com/higgsfield/RL-Adventure-2

        :inputs:
            :param states:          (list)  The observations recorded during the horizon.
            :param actions:         (list)  The actions taken during the horizon.
            :param old_log_probs:   (list)  The log probability of each action calculated during the horizon.
            :param returns:         (list)  The returns calculated during the horizon.
            :param old_values:      (list)  The estimate state values acquired during the horizon.
        :outputs:
            :return actor_loss:     (float) The loss value calculated for the actor.
            :return critic_loss:    (float) The loss value calculated for the critic.
        """

        # Calculate new values and log probabilities
        new_log_probs = []
        new_values = []
        entropy_term = 0
        for i in range(len(returns)):
            _, means, stds, entropy, value = self.get_action_and_value(states[i])
            log_prob = calculate_log_probability(actions[i], means, stds)

            new_log_probs.append(log_prob)
            new_values.append(value)
            entropy_term += entropy

        # Convert arrays into tensors and send them to the GPU to speed up calculations
        # frames = Variable(torch.FloatTensor(frames)).to(self.device)
        old_values = torch.FloatTensor(old_values).detach().to(self.device)
        new_values = Variable(torch.FloatTensor(new_values)).to(self.device)
        returns = torch.FloatTensor(returns).to(self.device)
        old_log_probs = torch.stack(old_log_probs).detach().to(self.device)
        new_log_probs = torch.stack(new_log_probs).to(self.device)

        # Calculate the advantage
        advantages = returns - old_values
        # advantages = Variable(advantages)

        # Compute the actor loss function with clipping
        ratio = (new_log_probs - old_log_probs).exp()
        loss_not_clipped = ratio * advantages
        loss_clipped = torch.clamp(ratio, 1.0 - self.clip_range, 1.0 + self.clip_range) * advantages
        actor_loss = -torch.min(loss_not_clipped, loss_clipped).mean() + 0.001 * entropy_term

        # Compute the critic loss function with clipping
        clipped_values = old_values + torch.clamp((new_values - old_values), -self.clip_range, self.clip_range)
        closs_clipped = (returns - clipped_values).pow(2)
        closs_not_clipped = (returns - new_values).pow(2)
        critic_loss = 0.5 * torch.max(closs_clipped, closs_not_clipped).mean()
        # critic_loss = 0.5 * (returns - new_values).pow(2).mean()

        # Combine for a total loss
        total_loss = actor_loss + critic_loss

        # Update optimizer
        self.ac_optimizer.zero_grad()
        total_loss.backward()
        self.ac_optimizer.step()

        # new_params = list(self.ac_nn.parameters())[0].clone()
        # print(torch.equal(new_params.data, self.old_params.data))
        #
        # self.old_params = new_params

        # Output the loss values for logging purposes
        return actor_loss.cpu(), critic_loss.cpu()


if __name__ == '__main__':
    rospy.init_node('ppo_control', anonymous=True)
    params = rospy.get_param("ppo_params")
    rospy.wait_for_service('/gazebo/reset_world')
    PPO_Controller = PPO(control_pub_name=params['control_pub_name'],
                         car_width=params['car_width'],
                         scan_width=params['scan_width'],
                         lidar_range=params['lidar_range'],
                         turn_clearance=params['turn_clearance'],
                         max_turn_angle=params['max_turn_angle'],
                         min_speed=params['min_speed'],
                         max_speed=params['max_speed'],
                         min_dist=params['min_dist'],
                         max_dist=params['max_dist'],
                         no_obst_dist=params['no_obst_dist'],
                         crash_threshold=params['crash_threshold'],
                         env=params['env'],
                         rate=params['rate'],
                         load_path=params['load_path'],
                         save_interval=params['save_interval'],
                         save_path=params['save_path'],
                         episode_length=params['episode_length'],
                         gamma=params['gamma'],
                         lam=params['lam'],
                         learning_rate=params['learning_rate'],
                         rl_clip_range=params['rl_clip_range'])
    rospy.Subscriber(params['lidar_name'], LaserScan, PPO_Controller.callback_lidar)
    rospy.Subscriber(params['ego_odom_name'], PoseStamped, PPO_Controller.callback_ego_odom)
    rospy.Subscriber(params['lead_odom_name'], PoseStamped, PPO_Controller.callback_leader_odom)
    time.sleep(10)
    PPO_Controller.test(-1)
