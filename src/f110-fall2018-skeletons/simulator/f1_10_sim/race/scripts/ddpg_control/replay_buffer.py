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
    def __init__(self, capacity, prev_buffer=None):
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
            self.position = 0
            self.capacity = float(capacity)

        else:
            fill = min(capacity, prev_buffer.capacity)
            self.states = prev_buffer.states[-fill:]
            self.actions = prev_buffer.actions[-fill:]
            self.rewards = prev_buffer.rewards[-fill:]
            self.dones = prev_buffer.dones[-fill:]
            self.next_states = prev_buffer.next_states[-fill:]
            self.position = int(prev_buffer.position % capacity)
            self.capacity = float(capacity)

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
        # If the buffer is not full, append it
        if len(self.rewards) < self.capacity:
            self.states.append(state)
            self.actions.append(action)
            self.rewards.append(reward)
            self.dones.append(done)
            self.next_states.append(next_state)
        # Otherwise, replace old memories
        else:
            self.states[self.position] = state
            self.actions[self.position] = action
            self.rewards[self.position] = reward
            self.dones[self.position] = done
            self.next_states[self.position] = next_state

        # Increment the position
        self.position = int((self.position + 1) % self.capacity)

    def sample_batch(self, batch_length):
        """
        TODO
        Assumes the buffer is full
        :param batch_length:
        :return:
        """
        # Convert lists to arrays
        states = np.asarray(self.states)
        actions = np.asarray(self.actions)
        rewards = np.asarray(self.rewards)
        dones = np.asarray(self.dones)
        next_states = np.asarray(self.next_states)

        # Randomize the order of the buffer to eliminate TODO
        indices = np.arange(self.capacity)
        np.random.shuffle(indices)

        # Collect the sample from the middle of the randomized indices
        batch_start = int(max((math.floor(self.capacity / 2.0) - math.floor(batch_length / 2.0)), 0))
        batch_end = int(min((math.floor(self.capacity / 2.0) + math.ceil(batch_length / 2.0)), self.capacity))
        batch_indeces = np.asarray(indices[batch_start:batch_end:1], dtype=int)
        # np.random.shuffle(batch_indeces)  # Shuffle the selected indices again to further randomize

        batch_states = states[batch_indeces]
        batch_actions = actions[batch_indeces]
        batch_rewards = rewards[batch_indeces]
        batch_dones = dones[batch_indeces]
        batch_next_states = next_states[batch_indeces]

        return batch_states, batch_actions, batch_rewards, batch_dones, batch_next_states


if __name__ == '__main__':
    replay_buffer = ReplayBuffer(10)
    s = np.asarray([0, 0, 0, 0])
    a = np.asarray([0, 0])
    r = 0
    d = 0
    for i in range(100):
        replay_buffer.add_memory(s, a, r, d, s)
        r += 1

    b_states, b_actions, b_rewards, b_dones, b_next_states = replay_buffer.sample_batch(19)
    print(b_rewards)

    new_buffer = ReplayBuffer(5, replay_buffer)
    new_buffer.add_memory(s, a, r, d, s)
    b_states, b_actions, b_rewards, b_dones, b_next_states = new_buffer.sample_batch(5)
    print('New Sample')
    print(b_rewards)

