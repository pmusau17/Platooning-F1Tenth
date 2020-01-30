"""
File:   class_cnn.py
Author: Nathaniel Hamilton

Description: This class file is where all Convolutional Neural Network (CNN) classes are kept.

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
class BodyNN(nn.Module):
    def __init__(self, input_size=4, hidden_size=6):
        super(BodyNN, self).__init__()
        """
        This Neural Network architecture makes up the body of the actor and critic NNs built in this class.
        
        :param input_size:  (int)   The desired size of the input layer. Should be the same size as the number of 
                                        inputs to the NN. Default=4
        :param hidden_size: (int)   The desired size of the hidden layers. Default=6
        """
        # Save the inputs
        self.input_size = input_size
        self.hidden_size = hidden_size

        # Establish the architecture of the NN body
        self.body = nn.Sequential(
            # Layer 1 (input -> hidden)
            nn.Linear(in_features=self.input_size, out_features=self.hidden_size),
            nn.LayerNorm(hidden_size),
            nn.ReLU(),
            # Layer 2 (hidden -> hidden)
            nn.Linear(in_features=self.hidden_size, out_features=self.hidden_size),
            nn.LayerNorm(hidden_size),
            nn.ReLU()
        )

    def initialize_orthogonal(self):
        """
        This function initializes the weights of the network according to the method described in "Exact solutions to
        the nonlinear dynamics of learning in deep linear neural networks" - Saxe, A. et al. (2013)
        """
        self.body.apply(init_weights)

    def forward(self, nn_input):
        """
        This function performs a forward pass through the network.

        :param nn_input:    (tensor)    The input the NN uses to compute an output.
        :return x:          (tensor)    The result after executing a forward pass through the network.
        """
        # Convert input into a Tensor and add an extra dimension
        #print('input size: ' + str(nn_input.size()))
        #x = Variable(nn_input.view(-1, self.input_size, 1))
        x = Variable(nn_input, requires_grad=True)

        # Pass the input through all the layers
        x = self.body(x)

        # Return the result
        return x


########################################################################################################################
class ActorCriticNN(nn.Module):
    def __init__(self, input_size=4, hidden_size=6, output_size=4):
        super(ActorCriticNN, self).__init__()
        """
        This Neural Network architecture creates a full actor, which provides the control output.
        
        :param input_size:  (int)   The desired size of the input layer. Should be the same size as the number of 
                                        inputs to the NN. Default=4
        :param hidden_size: (int)   The desired size of the hidden layers. Default=6
        :param output_size: (int)   The desired size of the output layer for the actor. The true output size of the NN 
                                        will be output_size+1 for the singular output of the critic. Default=4
        """
        # Save the inputs
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size

        # The main body shared by both the actor and the critic is the CNN developed in the Nature paper
        self.body = BodyNN()

        # Create the output layer for the actor
        self.fc_control = nn.Sequential(
            nn.Linear(in_features=self.hidden_size, out_features=self.output_size)
        )

        # Create the output layer for the critic
        self.fc_value_function = nn.Sequential(
            nn.Linear(in_features=self.hidden_size, out_features=1)
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

        # Initialize the critic
        self.fc_value_function.apply(init_weights)

    def forward(self, state):
        """
        This function performs a forward pass through the network.

        :param state:                   (tensor)    The input state the NN uses to compute an output.
        :return means:                  (tensor)    The means used for selecting the action.
        :return standard_deviations:    (tensor)    The standard deviations used for selecting the action.
        :return value:                  (tensor)    An estimate of the value of the input state.
        """
        # Don't need to modify the input state because that is already handled by the body's forward
        x = self.body.forward(state)

        # Take the result of the Body NN and pass it through the control layer
        mu = torch.tanh(self.fc_control(x))

        # Break apart the output into means and standard deviations
        means = mu[0:(self.output_size/2):1]
        standard_deviations = mu[(self.output_size/2):self.output_size:1]

        # Take the result of the Body NN and pass it through the value layer
        value = self.fc_value_function(x)

        # Return the result
        return means, standard_deviations, value
