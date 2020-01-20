"""
File:   nn_ddpg.py
Author: Nathaniel Hamilton

Description: This class file is where all Neural Networks (NN) architectures related to Deep Deterministic Policy
Gradient are kept as classes.

Usage:  Import the entire class file to use all options. Initialize CNNs as necessary.

Remaining Tasks:
        * Implement joint AC NN

"""

import numpy as np
import torch
import torch.nn as nn
from torch.autograd import Variable

# Set the random seed
seed_number = 8
np.random.seed(seed_number)
torch.manual_seed(seed_number)
torch.cuda.manual_seed_all(seed_number)


def init_weights(m):
    if type(m) == nn.Linear:
        torch.nn.init.orthogonal_(m.weight)


########################################################################################################################
class ActorNN(nn.Module):
    def __init__(self, input_size=4, hidden_size=6, output_size=4):
        super(ActorNN, self).__init__()
        """
        This Neural Network architecture creates a full actor, which provides the control output.

        :param input_size:  (int)   The desired size of the input layer. Should be the same size as the number of 
                                        inputs to the NN. Default=4
        :param hidden_size: (int)   The desired size of the hidden layers. Default=6
        :param output_size: (int)   The desired size of the output layer. Should be the same size as the number of 
                                        outputs from the NN. Default=4
        """
        # Save the inputs
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size

        # Initialize the body
        self.body = BodyNN(input_size=self.input_size, hidden_size=self.hidden_size)

        # Create the output layer
        self.fc_control = nn.Sequential(
            nn.Linear(in_features=self.hidden_size, out_features=self.output_size)
        )

    def initialize_orthogonal(self):
        """
        This function initializes the weights of the network according to the method described in "Exact solutions to
        the nonlinear dynamics of learning in deep linear neural networks" - Saxe, A. et al. (2013)
        """
        # Initialize the body
        self.body.initialize_orthogonal()

        # Initialize the actor
        self.fc_control.apply(init_weights)

    def forward(self, state):
        """
        This function performs a forward pass through the network.

        :param state:                   (tensor)    The input state the NN uses to compute an output.
        :return means:                  (tensor)    The means used for selecting the action.
        :return standard_deviations:    (tensor)    The standard deviations used for selecting the action.
        """
        # Don't need to scale images because that is already handled by nature's forward
        x = self.body.forward(state)

        # Take the result and pass it through the new layer
        x = self.fc_control(x)

        # Break apart the output into means and standard deviations
        means = x[0:1:1]
        standard_deviations = x[2:]

        # Return the result
        return means, standard_deviations


########################################################################################################################
class CriticNN(nn.Module):
    def __init__(self, input_size=4, hidden_size=6):
        super(CriticNN, self).__init__()
        """
        This Neural Network architecture creates a full critic, which estimates the value of a state.

        :param input_size:  (int)   The desired size of the input layer. Should be the same size as the number of 
                                        inputs to the NN. Default=4
        :param hidden_size: (int)   The desired size of the hidden layers. Default=6
        """
        # Save the inputs
        self.input_size = input_size
        self.hidden_size = hidden_size

        # Initialize the body
        self.body = BodyNN(input_size=self.input_size, hidden_size=self.hidden_size)

        # Create the output layer
        self.fc_value_function = nn.Sequential(
            nn.Linear(in_features=self.hidden_size, out_features=1),
            nn.ReLU
        )

    def initialize_orthogonal(self):
        """
        This function initializes the weights of the network according to the method described in "Exact solutions to
        the nonlinear dynamics of learning in deep linear neural networks" - Saxe, A. et al. (2013)
        """
        # Initialize the body
        self.body.initialize_orthogonal()

        # Initialize the critic
        self.fc_value_function(init_weights)

    def forward(self, state):
        """
        This function performs a forward pass through the network.

        :param state:                   (tensor)    The input state the NN uses to compute an output.
        :return x:                  (tensor)    An estimate of the value of the input state.
        """
        # Don't need to scale images because that is already handled by nature's forward
        x = self.body.forward(state)

        # Take the result and pass it through the new layer
        x = self.fc_value_function(x)

        # Return the result
        return x

