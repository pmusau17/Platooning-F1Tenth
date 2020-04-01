# import the layers to add in network fine-tuning
from keras.layers.core import Dropout
from keras.layers.core import Flatten
from keras.layers.core import Dense

class FCHeadNet:
    
    @staticmethod
    def build(baseModel,classes,D):
        # initialize the head model that will be placed on top of
        # the base then add a FC layer

        #get the shape for the new layers from the baseModel
        headModel = baseModel.output
        headModel = Flatten(name="flatten")(headModel)
        headModel = Dense(D,activation="relu")(headModel)
        headModel = Dropout(0.5)(headModel)

        # add a softmax layer

        headModel = Dense(classes,activation='softmax')(headModel)

        # return the model
        return headModel
        #result of this will be INPUT_NET => FC => RELU => DO => FC => SOFTMAX