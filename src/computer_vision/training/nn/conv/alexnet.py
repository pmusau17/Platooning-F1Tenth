# import the neccessary packages

from tensorflow.python.keras.models import Sequential
from tensorflow.python.keras.layers.normalization import BatchNormalization
from tensorflow.python.keras.layers.convolutional import Conv2D
from tensorflow.python.keras.layers.convolutional import MaxPooling2D

from tensorflow.python.keras.layers.core import Activation
from tensorflow.python.keras.layers.core import Flatten
from tensorflow.python.keras.layers.core import Dropout
from tensorflow.python.keras.layers.core import Dense
from tensorflow.keras.regularizers import l2 
from tensorflow.python.keras import backend as K 


class AlexNet:
    @staticmethod

    # reg controls the amount of L2 regularization that you'll be applying to the network
    # for larger and deeper networks applying regularization is critical to reducing overfitting

    def build(height,width,depth,classes,reg=0.002):

        # initialize the model along with the input shape to be
        # "channels last" and the channels dimension itself

        model= Sequential()
        inputShape= (height,width,depth)
        chanDim =-1

        # if we are using "channels first", update the input shape
        # and channels dimension

        if K.image_data_format() == "channels_first":
            inputShape = (depth,height,width)
            chanDim =1 

        # Block #1: first CONV => RELU => POOL layer set

        model.add(Conv2D(96,(11,11), strides = (4,4),input_shape=inputShape,padding="same",kernel_regularizer=l2(reg)))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(MaxPooling2D(pool_size=(3,3),strides=(2,2)))
        model.add(Dropout(0.25))

        # Block #2: second CONV => RELU => POOL layer set

        model.add(Conv2D(256,(5,5),padding="same",kernel_regularizer=l2(reg)))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(MaxPooling2D(pool_size=(3,3),strides=(2,2)))
        model.add(Dropout(0.25))

        # Deeper richer features are learned in the third block of AlexNet where we stack multiple CONV
        # => RELU together prior to applying a POOL operation

        # Block #3: CONV => RELU => CONV => RELU => CONV => RELU

        model.add(Conv2D(384,(3,3),padding="same",kernel_regularizer=l2(reg)))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(Conv2D(384,(3,3),padding="same",kernel_regularizer=l2(reg)))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(Conv2D(256,(3,3),padding="same",kernel_regularizer=l2(reg)))
        model.add(Activation("relu"))
        model.add(BatchNormalization(axis=chanDim))
        model.add(MaxPooling2D(pool_size=(3,3),strides=(2,2)))
        model.add(Dropout(0.25))

        # Block #4 first set of FC => RELU layers
        model.add(Flatten())
        model.add(Dense(4096,kernel_regularizer=l2(reg)))
        model.add(Activation("relu"))
        model.add(BatchNormalization())
        model.add(Dropout(0.5))

        # Block #5 second set of FC => RELU layers
        model.add(Dense(4096,kernel_regularizer=l2(reg)))
        model.add(Activation("relu"))
        model.add(BatchNormalization())
        model.add(Dropout(0.5))

        # softmax classifier
        model.add(Dense(classes,kernel_regularizer=l2(reg)))
        model.add(Activation("softmax"))

        # return the constructed network architechture
        return model 



