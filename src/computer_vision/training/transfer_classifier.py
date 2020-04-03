# import the neccessary packages

# import sys so we can use packages outside of this folder in
# either python 2 or python 3
import sys
#insert parent directory into the path
sys.path.insert(0,'..')

from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report
from nn.conv.fcheadnet import FCHeadNet
from preprocessing.utils import ImageUtils
# for data augmentation
from tensorflow.python.keras.preprocessing.image import ImageDataGenerator

#optimizers
from tensorflow.keras.optimizers import RMSprop
from tensorflow.keras.optimizers import SGD
from tensorflow.keras.applications import VGG16

# layers, model
from tensorflow.python.keras.layers import Input
from tensorflow.python.keras.models import Model 

# customary packages
import numpy as np 
import argparse
import os 

# construct the argument parser
ap = argparse.ArgumentParser()
ap.add_argument("-d","--dataset",required=True,help="path to input dataset")
ap.add_argument("-m","--model",required=True,help="path to save the output model")
args = vars(ap.parse_args())

# initialize some constants
HEAD_TUNING_EPOCHS = 25
FINAL_TUNING_EPOCHS = 100
BATCH_SIZE = 32
HEIGHT= 224
WIDTH= 224

# define the image augmentation generator
aug= ImageDataGenerator(rotation_range=5, brightness_range=[0.5,1.5], zoom_range=[0.9,1.1],rescale=1.0/255.0,fill_mode="nearest")

#load the data
iu=ImageUtils()
data,labels=iu.load_from_directory(args['dataset'],HEIGHT,WIDTH,verbose=1)

#normalize the images
data=data/255.0

#convert the labels from integers to vectors
lb = LabelBinarizer()
labels=lb.fit_transform(labels)

#since there are not an equal amount of images in each category lets compute the class totals
classTotals = labels.sum(axis=0).astype('float')
classWeight = classTotals.max() / classTotals
print(lb.classes_,classTotals,classWeight)

# partition the data into training and testing splits using 80% of
# the data for training and the remaining 20% for testing
#classification data
(trainX, testX, trainY, testY) = train_test_split(data,labels, test_size=0.20, stratify=labels, random_state=42)

# now comes the main event, the network surgery

baseModel = VGG16(weights='imagenet',include_top=False,input_tensor=Input(shape=(224,224,3)))
print(baseModel.summary())

# initialize the new head of the network, a set of FC layers
# followed by a softmax classifier
headModel=FCHeadNet.build(baseModel,testY.shape[1],256)

# place the head FC model on top of the base model -- this will 
# become the actual model we will train

model = Model(inputs=baseModel.input,outputs=headModel)

print(model.summary())

# loop over all layers in the base model and freeze them so they 
# will * not* be updated during the training process.

for layer in baseModel.layers:
    layer.trainable = False

# compile our model. This needs to be done after our setting our
# layers to being non-trainable

print("[INFO] compiling model...")
opt = RMSprop(lr=0.001)
model.compile(loss='categorical_crossentropy',optimizer=opt, metrics=['accuracy'])

# train the head of the network for a few epochs (all other layers
# are frozen) -- this will allow the new FC layers to start to become 
# initialized with actual learned values verus being purely random

print ('[INFO] training head...')
model.fit_generator(aug.flow(trainX,trainY,batch_size=BATCH_SIZE),
                    validation_data=(testX,testY),epochs=HEAD_TUNING_EPOCHS,
                    steps_per_epoch=len(trainX)//HEAD_TUNING_EPOCHS,verbose=1)

# evaluate the network after initialization 
print("[INFO] evaluating after initialization")
predictions = model.predict(testX, batch_size = BATCH_SIZE)
print(classification_report(testY.argmax(axis=1),predictions.argmax(axis=1),target_names=classNames))

# now that the head FC layers have been trained/initialized, lets 
# unfreeze the final set of CONV layers and make them trainable
# the only reason that we didn't unfreeze the whole thing is that 
# VGG is a deep architechture, if classification accuracy continues
# to improve (wihtout overfitting), you may consider unfreezing more 
# layers in the body
for layer in baseModel.layers[15:]:
    layer.trainable = True


# for the changes to the model to take affect we need to recompile the
# model, this time using SGD with a *very* small learning rate

print("[INFO] re-compiling model...")
opt = SGD(lr=0.001)
model.compile(loss='categorical_crossentropy',optimizer=opt,metrics=['accuracy'])

print ('[INFO] fine-tuning model...')
model.fit_generator(aug.flow(trainX,trainY,batch_size=BATCH_SIZE),
                    validation_data=(testX,testY),epochs=FINAL_TUNING_EPOCHS,
                    steps_per_epoch=len(trainX)//FINAL_TUNING_EPOCHS,verbose=1)

# evaluate the network after initialization 
print("[INFO] evaluating after fine-tuning")
predictions = model.predict(testX, batch_size = BATCH_SIZE)
print(classification_report(testY.argmax(axis=1),predictions.argmax(axis=1),target_names=classNames))

#save te model to disk
print("[INFO] serializing model...")
model.save(args['model'])
