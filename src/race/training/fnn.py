#import the necessary packages

from keras import Sequential

#The activation package handles applying an activation to an input
#The Flatten classes takes our multi-dimensional volume and flattens it into a 1D array prior to feeding the inputs into the Dense (i.e, fully-connected) layers

from keras.layers.core import Activation, Flatten, Dense
from keras import Input
#implement your neural networks in a class to keep your code organized

class FNN:


    #width: The width of the input images that will be used to train the network
    #height: The height of our input images
    #depth: The number of channels in the input image
    #classes: The total number of classes that our network should learn to predict
    #this is a static method: notice it doesn't take the self parameter

    #These lines should be in every CNN you train so that your network will work regardless of how a user is ordering 
    #the channels of their image
    @staticmethod
    def build(inputShape,classes,neurons):
       

        # define sequential model with two layers of 64 neurons
        model = Sequential()
        model.add(Dense(neurons,input_dim=inputShape, init="uniform"))
        model.add(Activation("relu"))
        model.add(Dense(neurons))
        model.add(Activation("relu"))

        model.add(Dense(classes))
        model.add(Activation("tanh"))

        #return the constructed network architechture
        return model
        


if __name__ == "__main__":
    print(FNN.build((10,),1,64).summary())
