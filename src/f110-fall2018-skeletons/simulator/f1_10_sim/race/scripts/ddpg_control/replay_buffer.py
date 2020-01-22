"""
File:   replay_buffer.py
Author: Nathaniel Hamilton

Description: This class implements a replay buffer where the relevant information of past experiences is stored and can
             be sampled from.

Usage:  Import the entire class file to use all options.

Remaining Tasks:
        * Make it

"""
import numpy as np
import math


class ReplayBuffer(object):
    def __init__(self, prev_buffer=None):
        """
        TODO
        """
        # Initialize the replay buffer to empty if no previous buffer is supplied
        if prev_buffer is None:
            self.states = []
            self.actions = []
            self.rewards = []
            self.dones = []
            self.next_states = []
            self.length = 0

        else:
            self.states = prev_buffer.states
            self.actions = prev_buffer.actions
            self.rewards = prev_buffer.rewards
            self.dones = prev_buffer.dones
            self.next_states = prev_buffer.next_states
            self.length = prev_buffer.length

    def add_memory(self, state, action, reward, done, next_state):
        """
        TODO
        :param state:
        :param action:
        :param reward:
        :param done:
        :param next_state:
        :return:
        """
        # Append the new memory to the end and increase the length
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.dones.append(done)
        self.next_states.append(next_state)
        self.length += 1.0

    def sample_batch(self, batch_length):
        """
        TODO
        :param batch_length:
        :return:
        """
        # Convert fuck all
        states = np.asarray(self.states)
        actions = np.asarray(self.actions)
        rewards = np.asarray(self.rewards)
        dones = np.asarray(self.dones)
        next_states = np.asarray(self.next_states)

        # Determine the sample size (cannot sample more than the length)
        sample_size = min(self.length, batch_length)

        # Randomize the order of the buffer to eliminate TODO
        indeces = np.arange(self.length)
        np.random.shuffle(indeces)

        # Collect the sample from the middle of the randomized indeces
        batch_start = int(max((math.floor(self.length / 2.0) - math.floor(sample_size / 2.0)), 0))
        batch_end = int(min((math.floor(self.length / 2.0) + math.ceil(sample_size / 2.0)), self.length))
        batch_indeces = np.asarray(indeces[batch_start:batch_end:1], dtype=int)
        np.random.shuffle(batch_indeces)  # Shuffle the selected indices again to further randomize

        batch_states = states[batch_indeces]
        batch_actions = actions[batch_indeces]
        batch_rewards = rewards[batch_indeces]
        batch_dones = dones[batch_indeces]
        batch_next_states = next_states[batch_indeces]

        return batch_states, batch_actions, batch_rewards, batch_dones, batch_next_states


if __name__ == '__main__':
    replay_buffer = ReplayBuffer()
    s = np.asarray([0, 0, 0, 0])
    a = np.asarray([0, 0])
    r = 0
    d = 0
    for i in range(100):
        replay_buffer.add_memory(s, a, r, d, s)
        r += 1

    b_states, b_actions, b_rewards, b_dones, b_next_states = replay_buffer.sample_batch(19)
    print(b_rewards)

    new_buffer = ReplayBuffer(replay_buffer)
    b_states, b_actions, b_rewards, b_dones, b_next_states = new_buffer.sample_batch(50)
    print('New Sample')
    print(b_rewards)

