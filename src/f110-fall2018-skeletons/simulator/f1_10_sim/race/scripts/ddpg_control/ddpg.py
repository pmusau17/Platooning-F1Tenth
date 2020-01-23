#!/usr/bin/env python
"""
File:   ddpg.py
Author: Nathaniel Hamilton

Description: blah
    State is an array with the following information:
        [forward distance to lead vehicle, sideways distance to lead vehicle, speed_lead - speed_ego, yaw_lead - yaw_ego]
    Action is an array with the following information:
        [desired speed, desired steering angle]

Usage:  blah

Class Functions:
    * get_action(array)
    * get_state()
    * learn(int, int, int, int) (kinda done)
    * publish_command(float, float)
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
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import time
import gc
import rospy
import math
import os
import numpy as np
import torch
import torch.optim as optim
from torch.autograd import Variable
from tf.transformations import euler_from_quaternion

from nn_ddpg import *
from replay_buffer import *
from noise_OU import *

from race.msg import drive_param  # For simulator
# from racecar.msg import drive_param # For actual car

# Set the random seed
seed_number = 8
np.random.seed(seed_number)
torch.manual_seed(seed_number)
torch.cuda.manual_seed_all(seed_number)


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


def calculate_reward(curr_state, done):
    """

    :param curr_state:  (ndarray)   Current state of the agent
    :param done:        (int)       Variable determining if the vehicle crashed or not
    :return reward:     (float)     The reward associated with the state information
    """
    if done == 1:
        reward = 0
    else:
        goal_state = np.array([0.5, 0.0, 0.0, 0.0])
        mask = np.array([2.0, 1.0, 0.0, 0.0])

        diff = goal_state - curr_state
        diff = np.multiply(diff, mask)
        diff = np.absolute(diff)
        reward = 10 - np.sum(diff)

    # print(curr_state)
    # print(reward)

    return reward


def soft_update(target, source, tau):
    """
    TODO
    :param target:
    :param source:
    :param tau:
    :return:
    """

    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)


def hard_update(target, source):
    """
    TODO
    :param target:
    :param source:
    :return:
    """

    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(param.data)


def reset_env():
    """
    TODO: write this
    """
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    reset_world()
    # print("The world has been reset")
    return


class DDPG(object):
    def __init__(self, control_pub_name,
                 max_turn_angle=34.0, min_speed=0.5, max_speed=4.0, min_dist=0.1, max_dist=3.0, crash_threshold=10,
                 env='sim', rate=20, load_path=None, log_path='./', save_interval=10, save_path='./',
                 episode_length=8192, replay_capacity=8192, batch_size=100, actor_learning_rate=1e-4, critic_learning_rate=1e-3,
                 weight_decay=1e-2, gamma=0.99, tau=0.001):
        """

        """

        # Save the input parameters
        self.max_turn_angle = max_turn_angle
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.min_dist = min_dist
        self.max_dist = max_dist
        self.crash_threshold = crash_threshold
        self.env = env
        self.rate = rospy.Rate(rate)  # TODO: Look into this. might want to be a constant wait time
        self.load_path = load_path
        self.log_path = log_path
        self.save_intv = save_interval
        self.save_path = save_path
        self.episode_length = episode_length
        self.capacity = replay_capacity
        self.batch_size = batch_size
        self.actor_lr = actor_learning_rate
        self.critic_lr = critic_learning_rate
        self.weight_decay = weight_decay
        self.gamma = gamma
        self.tau = tau

        # Initialize publishers
        self.pub_drive_param = rospy.Publisher(control_pub_name, drive_param, queue_size=5)

        # Initialize Cuda variables
        use_cuda = torch.cuda.is_available()
        self.device = torch.device("cuda" if use_cuda else "cpu")

        # Create the actor and critic neural network
        self.actor_nn = ActorNN().to(self.device)
        self.critic_nn = CriticNN().to(self.device)

        # Create the target networks
        self.actor_target_nn = ActorNN().to(self.device)
        self.critic_target_nn = CriticNN().to(self.device)

        # Create the optimizers for the actor and critic neural networks
        self.actor_optimizer = optim.Adam(self.actor_nn.parameters(), lr=self.actor_lr)
        self.critic_optimizer = optim.Adam(self.critic_nn.parameters(), lr=self.critic_lr,
                                           weight_decay=self.weight_decay)

        # Initialize the target NNs or load models
        if load_path is None or load_path == 'None':
            # Targets are copied with a hard update
            hard_update(self.actor_target_nn, self.actor_nn)
            hard_update(self.critic_target_nn, self.critic_nn)

        else:
            self.load_models(self.load_path)

        # Initialize variables used by subscribers
        self.ego_pos = None  # [0.0, 0.0, 0.0, 0.0]  # (x, y, yaw, speed)
        self.lead_pos = None  # [(max_dist - min_dist)/2, 0.0, 0.0, 0.0]   # (x, y, yaw, speed)
        self.lidar_done = 0

        # Initialize variables used by the learner
        self.start_time = 0
        self.replay_buffer = ReplayBuffer(self.capacity)
        self.noise = OrnsteinUhlenbeckActionNoise(mu=np.asarray([0, 0]), sigma=self.sigma, theta=self.theta,
                                                  dt=(1.0/float(rate)))
        self.scale_mult = np.asarray([((self.max_speed - self.min_speed) / 2.0), self.max_turn_angle], dtype=float)
        self.scale_add = np.asarray([((self.max_speed - self.min_speed) / 2.0), 0.0], dtype=float)

    def callback_ego_odom(self, data):
        """

        """
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w

        quaternion = (qx, qy, qz, qw)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        dx = data.twist.twist.linear.x
        dy = data.twist.twist.linear.y
        speed = math.sqrt(dx ** 2 + dy ** 2)

        self.ego_pos = [x, y, yaw, speed]
        # print('ego_pos updated')

    def callback_leader_odom(self, data):
        """

        """
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w

        quaternion = (qx, qy, qz, qw)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        dx = data.twist.twist.linear.x
        dy = data.twist.twist.linear.y
        speed = math.sqrt(dx ** 2 + dy ** 2)

        self.lead_pos = [x, y, yaw, speed]
        # print('lead_pos updated')

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

        # print('Lidar updated')

    def get_action(self, state, noise=np.asarray[0, 0]):
        """
        This function calculates the desired output using the NN and the exploration noise.

        :input:
            :param state:       (ndarray)   Input state to determine which action to take
            :param noise:       (ndarray)   The exploration noise. Default=[0, 0] (no noise)
        :outputs:
            :return action:     (ndarray)   The chosen action to take
        """
        # Forward pass the network
        state = torch.FloatTensor(state).to(self.device)
        self.actor_nn.eval()  # Must be in eval mode to execute a forward pass
        action = self.actor_nn.forward(state)
        self.actor_nn.train()  # Must be in train mode to record gradients
        action = action.cpu()

        # Add the process noise to the action
        action = action + torch.FloatTensor(noise).cpu()
        action = action.clamp(-1.0, 1.0)  # Make sure action cannot exceed limits

        # Convert to numpy array
        action.detach().numpy()

        return action

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

    def learn(self, total_steps=int(1e9), test_length=8192, num_tests=4):
        """

        """

        # Make sure the Neural Nets are in train mode
        self.set_nns_to_train()

        # Create the log files
        if not os.path.isdir(self.log_path):
            os.mkdir(self.log_path)
        episode_performance_save_string = self.log_path + '/episode_performance.csv'
        f = open(episode_performance_save_string, "w+")
        f.write("training steps, average reward per step, chance to crash \n")
        f.close()

        # Iterate through a sufficient number of steps broken into horizons
        step_count = 0
        save_count = 0
        while step_count < total_steps:
            ep_length = min(self.episode_length, (total_steps - step_count))
            # Train through exploration
            self.play_through_training(ep_length)

            # Evaluate performance periodically
            avg_ep_reward, chance_to_crash = self.test(test_length, num_tests)

            # Record the episode information for logging
            if self.log_path is None:
                # Print to the console if no log path is specified
                print(
                    'Step Count: ' + str(step_count) + ' Avg reward per step: ' + str(avg_ep_reward) +
                    ' Chance to Crash: ' + str(chance_to_crash))
            else:
                # Write the performance after testing to a file
                with open(episode_performance_save_string, "a") as myfile:
                    myfile.write(str(step_count) + ', ' + str(avg_ep_reward) + ', ' + str(chance_to_crash) + '\n')

                # Print to the console
                print('Step Count: ' + str(step_count) + ' Avg reward per step: ' + str(avg_ep_reward)
                      + ' Chance to Crash: ' + str(chance_to_crash))

            # Increment the step counter
            step_count += ep_length

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
            self.start_time = checkpoint['time_step'] + 1
            self.actor_nn.load_state_dict(checkpoint['actor'])
            self.critic_nn.load_state_dict(checkpoint['critic'])
            self.actor_target_nn.load_state_dict(checkpoint['actor_target'])
            self.critic_target_nn.load_state_dict(checkpoint['critic_target'])
            self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer'])
            self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer'])
            self.replay_buffer = ReplayBuffer(checkpoint['replay_buffer'])

            # Evaluate the neural network to ensure the weights were properly loaded
            self.actor_nn.eval()
            self.critic_nn.eval()

        # Clean up any garbage that's accrued
        gc.collect()

        return

    def play_through_training(self, episode_length):
        """

        """

        # Initialize variables
        steps_taken = 0
        states = []
        actions = []
        log_probs = []
        returns = []
        values = []

        ep_steps = []
        ep_rewards = []
        ep_dones = []

        # Make sure the environment starts fresh if in simulation
        if self.env == 'sim':
            reset_env()
            self.rate.sleep()
            print('New Horizon')
            self.lidar_done = 0

        # Play through episodes and record the results until the horizon has been played through
        while steps_taken < horizon_length:
            # Determine how many steps to take in the next episode
            max_steps = min((horizon_length - steps_taken), self.episode_length)

            # Execute an episode (returns if done or reached desired number of steps)
            ep_states, ep_actions, ep_log_probs, ep_returns, ep_values, steps, ep_reward, done = self.test(max_steps)

            # Update the records
            steps_taken += steps
            states.extend(ep_states)
            actions.extend(ep_actions)
            log_probs.extend(ep_log_probs)
            returns.extend(ep_returns)
            values.extend(ep_values)

            # Record information about the episode for logging
            ep_steps.append(steps_taken)
            ep_rewards.append(ep_reward)
            ep_dones.append(done)

            # Only reset the environment if a terminal state has been reached
            if done == 1:
                if self.env == 'sim':
                    reset_env()
                    self.rate.sleep()
                    print(steps_taken)
                    self.lidar_done = 0

        # print(len(states))

        states = np.asarray(states)
        # print(len(states))
        actions = np.asarray(actions)
        log_probs = np.asarray(log_probs)
        returns = np.asarray(returns)
        values = np.asarray(values)

        return states, actions, log_probs, returns, values, ep_steps, ep_rewards, ep_dones

    def publish_cmd(self, velocity, steering_angle):
        """

        :param velocity:        (float)
        :param steering_angle:  (float)
        """

        msg = drive_param()
        msg.angle = steering_angle
        msg.velocity = velocity
        self.pub_drive_param.publish(msg)

        return

    def step(self, action):
        """
        TODO
        :param action:      (ndarray)
        """
        # Scale the action
        action = np.multiply(action, self.scale_mult) + self.scale_add

        # Publish action
        vel_cmd = action[0]
        steer_cmd = action[1]
        self.publish_cmd(vel_cmd, steer_cmd)

        # Wait specified time
        self.rate.sleep()

        # Collect new state
        next_state, done = self.get_state()
        reward = calculate_reward(next_state, done)

        return next_state, reward, done

    def save_models(self, time_step, save_path='models.pth'):
        """
        This function save the neural network models and optimizers for both the actor and the critic in one file. For
        more examples on how to save/load models, visit
        https://pytorch.org/tutorials/beginner/saving_loading_models.html

        :param time_step:     (int)
        :param save_path:     (string) The file name that the models will be saved to. default='models.pth'
        """

        # Save all aspects of the learner in one file
        torch.save({
            'time_step': time_step,
            'actor': self.actor_nn.state_dict(),
            'critic': self.critic_nn.state_dict(),
            'actor_target': self.actor_target_nn.state_dict(),
            'critic_target': self.critic_target_nn.state_dict(),
            'actor_optimizer': self.actor_optimizer.state_dict(),
            'critic_optimizer': self.critic_optimizer.state_dict(),
            'replay_buffer': self.replay_buffer,
        }, save_path)

        # Clean up any garbage that's accrued
        gc.collect()

        return

    def test(self, test_length=-1, num_tests=1):
        """
        This function runs the learned policy until a stop condition occurs.

        :inputs:
            :param test_length:   (int) Maximum number of steps to execute in the training cycle. If -1, then there is
                                          no maximum. Default=-1
            :param num_tests:     (int) Number of tests to be executed. Default=1
        """

        # Initialize a count of the number of times the vehicle crashes
        crashes = 0

        avg_episode_reward = 0

        for _ in range(num_tests):
            # Initialize for a new test episode
            if self.env == 'sim':
                reset_env()
                self.rate.sleep()
                self.lidar_done = 0
            state, done = self.get_state()
            step = 0
            total_reward = 0

            # Execute all the steps in the episode
            while not rospy.is_shutdown():
                # Stop the controller if there is a collision or time-out
                if done or (step >= test_length != -1):
                    # stop
                    self.publish_cmd(0.0, 0.0)
                    crashes += done
                    break

                # Determine the next action
                action = self.get_action(state)

                # Execute determined action
                next_state, reward, done = self.step(action)

                # Update for next step
                total_reward += reward
                state = next_state
                step += 1

            # Compute the average reward for the episode
            if step == 0:
                avg_reward = 0
            else:
                avg_reward = total_reward / step

            avg_episode_reward += avg_reward/num_tests

        # Compute the chance of crashing based on the number of tests run
        chance_to_crash = crashes / num_tests

        return avg_episode_reward, chance_to_crash

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
        new_values = Variable(torch.FloatTensor(new_values), requires_grad=True).to(self.device)
        returns = torch.FloatTensor(returns).to(self.device)
        old_log_probs = torch.FloatTensor(old_log_probs).detach().to(self.device)
        new_log_probs = Variable(torch.FloatTensor(new_log_probs), requires_grad=True).to(self.device)

        # Calculate the advantage
        advantages = returns - old_values
        advantages = Variable(advantages, requires_grad=True)

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
        total_loss = Variable(total_loss, requires_grad=True)

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
                         log_path=params['log_path'],
                         save_interval=params['save_interval'],
                         save_path=params['save_path'],
                         episode_length=params['episode_length'],
                         gamma=params['gamma'],
                         lam=params['lam'],
                         learning_rate=params['learning_rate'],
                         rl_clip_range=params['rl_clip_range'])
    rospy.Subscriber(params['lidar_name'], LaserScan, PPO_Controller.callback_lidar)
    rospy.Subscriber(params['ego_odom_name'], Odometry, PPO_Controller.callback_ego_odom)
    rospy.Subscriber(params['lead_odom_name'], Odometry, PPO_Controller.callback_leader_odom)
    PPO_Controller.rate.sleep()
    if params['test_or_train'] == 'train':
        # Train the network
        PPO_Controller.learn(horizon_length=params['horizon_length'],
                             num_epochs=params['num_epochs'],
                             minibatch_length=params['minibatch_length'])
    else:
        if params['env'] == 'sim':
            reset_env()
            PPO_Controller.rate.sleep()
            # PPO_Controller.ac_nn.eval()
            _, _, _, _, _, _, avg_reward, _ = PPO_Controller.test(-1)
            print('Average Reward per Step: ' + str(avg_reward))
        else:
            PPO_Controller.test(-1)
