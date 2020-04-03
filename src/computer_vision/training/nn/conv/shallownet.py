#import the necessary packages

from tensorflow.python.keras import Sequential
from tensorflow.python.keras.layers.convolutional import Conv2D #this is the Keras implementation of the convolutional layer we have discussed up to this point

#The activation package handles applying an activation to an input
#The Flatten classes takes our multi-dimensional volume and flattens it into a 1D array prior to feeding the inputs into the Dense (i.e, fully-connected) layers

from tensorflow.python.keras.layers.core import Activation, Flatten, Dense
from tensorflow.python.keras.backend import image_data_format

#implement your neural networks in a class to keep your code organized

class ShallowNet:


    #width: The width of the input images that will be used to train the network
    #height: The height of our input images
    #depth: The number of channels in the input image
    #classes: The total number of classes that our network should learn to predict
    #this is a static method: notice it doesn't take the self parameter

    #These lines should be in every CNN you train so that your network will work regardless of how a user is ordering 
    #the channels of their image
    @staticmethod
    def build(width, height, depth, classes):
        # initialize the model along with the input shape to be
        # "channels last"

        model = Sequential()
        #the image input
        inputShape = (height, width, depth)

        # if we are using "channels first", update the input shape
        if image_data_format() == "channels_first":
            inputShape = (depth, height, width)

        #Every CNN that you implement will have a build method this function will accept a
        #number of parameters, construct the network architecture, and then return it to the calling function
        #It will accept a number of parameters

        #define the first (and only) CONV=>RELU layer
        #This layer will have 32 filters each of which are 3x3, apply the asame padding 
        #to ensure the size of the output of the convolution operations matches the input
        #(using same padding isn't strictly neccessary for this example, but it's a good)
        #habbit to start forming now
        model.add(Conv2D(32,(3,3),padding="same"))
        model.add(Activation("relu"))

        #softmax classifier
        model.add(Flatten())
        model.add(Dense(classes))
        model.add(Activation("softmax"))

        #return the constructed network architechture
        return model
        



