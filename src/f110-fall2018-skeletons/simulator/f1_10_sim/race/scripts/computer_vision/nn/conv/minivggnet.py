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


#Implement the architechture with a build model 

class MiniVGGNet:
    @staticmethod
    def build(height,width,depth,classes):
        #initialize the model along with the input shape to be 
        #channel's last and the channels dimension itself
        model = Sequential()

        #shape of the input images
        inputShape=(height,width,depth)
        chanDim=-1

        #if we are using "channels first", update the input shape 
        #and channels dimension. The reason that have to store the chanDim dimension
        #is in order to apply Batch Norm. Batch Norm operates over channels 

        if K.image_data_format() == "channels_first":
            inputShape=(depth,height,width)
            chanDim=1

        #first block of MiniVGGNet

        model.add(Conv2D(32,(3,3),padding="same",input_shape=inputShape))
        model.add(Activation('relu'))
        model.add(BatchNormalization(axis=chanDim))
        model.add(Conv2D(32,(3,3),padding="same",))
        model.add(Activation('relu'))
        model.add(BatchNormalization(axis=chanDim))
        #in keras when you don't explicitly set the size it assumes
        #that the stride length will be equal to the max pooling size
        model.add(MaxPooling2D(pool_size=(2,2)))
        model.add(Dropout(0.25))
        
        #second block of MiniVGGNet, This is the same as the above block but we are
        #increasing the number of filters which is very common in CNNs
        model.add(Conv2D(64,(3,3),padding="same"))
        model.add(Activation('relu'))
        model.add(BatchNormalization(axis=chanDim))
        model.add(Conv2D(64,(3,3),padding="same",))
        model.add(Activation('relu'))
        model.add(BatchNormalization(axis=chanDim))
        model.add(MaxPooling2D(pool_size=(2,2)))
        model.add(Dropout(0.25))

        #first (and only) set of FC => RELU layers
        #typically you see dropout with p=0.5 in between 
        #FC layers
        model.add(Flatten())
        model.add(Dense(512))
        model.add(Activation('relu'))
        model.add(BatchNormalization())
        model.add(Dropout(0.50))

        #softmax classifier
        model.add(Dense(classes))
        model.add(Activation("softmax"))

        #now that we've implemented mini VGGNet Architecthure
        return model

