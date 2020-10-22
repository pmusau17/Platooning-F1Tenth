# import sys so we can use packages outside of this folder in
# either python 2 or python 3
import sys
#insert parent directory into the path
sys.path.insert(0,'..')


#import the necessary sklearn packages
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report

#import minivggnet, shallownet
from nn.conv.minivggnet import MiniVGGNet
from nn.conv.shallownet import ShallowNet
from preprocessing.utils import ImageUtils

#import tensorflow libraries
from tensorflow.python.keras.preprocessing.image import img_to_array
from tensorflow.python.keras.utils import np_utils
from tensorflow.python.keras.callbacks import ModelCheckpoint
import tensorflow.keras.backend as K  
from tensorflow.keras.optimizers import SGD
from tensorflow.keras.callbacks import EarlyStopping

#import imutils to handle resizing and path importing
from imutils import paths
#import other useful libraries
import matplotlib.pyplot as plt
import numpy as np
import argparse
import imutils
import cv2
import os


#performs grid search over epochs and batches
def train_model(model,X_train,X_test,Y_train,Y_test,classWeight,num_epochs,batch_size,learning_rate):
    #optimizer
    opt=SGD(lr=learning_rate,decay=0.01/num_epochs,momentum=0.9,nesterov=True)
    #compile the model
    model.compile(loss='categorical_crossentropy',optimizer=opt,metrics=['accuracy'])
    #callbacks
    #checkpoint = ModelCheckpoint(fname, monitor="val_loss", mode="min",save_best_only=True,save_weights_only=False, verbose=1)
    callbacks_list = [EarlyStopping(monitor='loss', patience=10, verbose=0)]
    #train the model
    H = model.fit(X_train, Y_train, validation_data=(X_test, Y_test),class_weight=classWeight, batch_size=128, epochs=num_epochs, verbose=1,callbacks=callbacks_list)
    return H


# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dataset", required=True,help="path to input dataset")
ap.add_argument("-t",'--trials',required=True,type=int,help="number of trials to run")
args = vars(ap.parse_args())

#desired height and width these come from the classifier model
height= 32
width= 32

#Image Utils
iu=ImageUtils()
data,labels=iu.load_from_directory(args['dataset'],height,width,verbose=1)
data=data/255.0

# convert the labels from integers to vectors
#THIS WOULD HAVE SOLVED MY PROBLEM WEEKS AGO
lb = LabelBinarizer()
labels=lb.fit_transform(labels)

#since there are not an equal amount of images in each category lets compute the class totals
classTotals = labels.sum(axis=0).astype('float')
classWeight = classTotals.max() / classTotals
print(lb.classes_,classTotals,classWeight)

# partition the data into training and testing splits using 80% of
# the data for training and the remaining 20% for testing
#classification data

#This stratify parameter makes a split so that the proportion of values
# in the sample produced will be the same as the proportion of values 
# provided to parameter stratify
(trainX, testX, trainY, testY) = train_test_split(data,labels, test_size=0.20, stratify=labels, random_state=42)

# initialize the model
print("[INFO] compiling model...")
model=MiniVGGNet.build(height=height,width=width,depth=3,classes=len(lb.classes_))

#upper and lower ranges
epochs = [10,250]
batches = [16,512]
learning_rates=[0.001,0.1]

grid={}
#do the random search
for learning_rate in learning_rates:

    epoch_size=np.random.randint(epochs[0],epochs[1])
    batch_size=np.random.randint(batches[0],batches[1])
    learning_rate=np.random.uniform(learning_rates[0],learning_rates[1])
            
    key=str(epoch_size)+'-'+str('batch_size')+'-'+str(learning_rate)
    print('------epoch: {}, batch_size: {}, learning_rate: {}--------'.format(epoch_size,batch_size,learning_rate))

    #train the model
    H=train_model(model,trainX, testX, trainY, testY,classWeight,epoch_size,batch_size,learning_rate)

    #loss
    minimum_loss=np.min(H.history['val_loss'])
    grid[key]=minimum_loss
    print("Validation Loss:{}".format(minimum_loss))
            
print(grid)
