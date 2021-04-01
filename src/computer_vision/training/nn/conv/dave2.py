#import the neccessary kpackages
from tensorflow.python.keras.models import Sequential
from tensorflow.python.keras.layers.normalization import BatchNormalization
from tensorflow.python.keras.layers.convolutional import Conv2D
from tensorflow.python.keras.layers.convolutional import MaxPooling2D
from tensorflow.python.keras.layers.core import Activation 
from tensorflow.python.keras.layers.core import Flatten
from tensorflow.python.keras.layers.core import Dropout
from tensorflow.python.keras.layers.core import Dense
from tensorflow.python.keras import backend as K
from tensorflow.keras.utils import plot_model

#Builds the DAVE2 Architechture described in: https://devblogs.nvidia.com/deep-learning-self-driving-cars/
class DAVE2: 
    @staticmethod
    def build(height,width,depth,classes):
        # initialize the model
        model = Sequential()
        #the shape of our image inputs
        inputShape=(height, width, depth)
        #if we are using "channels first" update the input shape
        if (K.image_data_format()=="channels_first"):
            inputShape=(depth, height, width)

        #first set of CONV=>RELU=> POOL layers
        model.add (Conv2D(24,(5,5),strides=(2,2), padding="valid",input_shape=inputShape))
        model.add(Activation('elu'))

        #second set of 5x5 CONV=>RELU=> POOL layers
        model.add (Conv2D(36,(5,5), strides=(2,2),padding="valid"))
        model.add(Activation('elu'))

        #third set of 5x5 CONV=>RELU=> POOL layers
        model.add (Conv2D(48,(5,5), strides=(2,2),padding="valid"))
        model.add(Activation('elu'))

        #first set of 3x3 CONV=>RELU=> POOL layers
        model.add (Conv2D(64,(3,3), padding="valid"))
        model.add(Activation('elu'))

        #second set of 3x3 CONV=>RELU=> POOL layers
        model.add (Conv2D(64,(3,3), padding="valid"))
        model.add(Activation('elu'))

        #set of fully connected layers
        model.add(Flatten())
        model.add(Dense(1164))
        model.add(Activation('elu'))
        model.add(Dense(100))
        model.add(Activation('elu'))
        model.add(Dense(50))
        model.add(Activation('elu'))
        model.add(Dense(10))
        model.add(Activation('elu'))

        #output
        model.add(Dense(classes))
        model.add(Activation('tanh'))

        return model

if __name__=="__main__":
    model=DAVE2.build(66,200,3,1)
    print(model.summary())
    #This is responsible for constructing a graph based on the layers inside 
    #The input model and then writing the graph to disk an image
    plot_model(model, to_file="DAVE3.png",show_shapes=True)