#import the necessary sklearn packages
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report

#import tensorflow packages
from tensorflow.python.keras.preprocessing.image import img_to_array
from tensorflow.python.keras.utils import np_utils
from tensorflow.python.keras.models import load_model
from tensorflow.python.keras import backend as K 

#import imutils to handle resizing and path importing
import imutils

#numpy, argparse, cv2
import numpy as np 
import argparse
import cv2
import os

#import the preprocessing utils (helps with loading data, preprocessing)
from preprocessing.utils import ImageUtils

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dataset", required=True,help="path to input dataset")
ap.add_argument("-m", "--model", required=True,help="path to the model we are loading")
args = vars(ap.parse_args())



#desired height and width these come from the DAVE model
height= 32
width= 32

#Image Utils
iu=ImageUtils()
data,labels=iu.load_from_directory(args['dataset'],height,width,verbose=1)

#Normalize the data, shuffle it, and binarize the labels
data=data/255.0
lb = LabelBinarizer()
labels=lb.fit_transform(labels)
print(lb.classes_)
np.random.shuffle(data)


#load the pre-trained network (Keras models are HDF5 models)
print('[INFO] loading pre-trained network...')
model= load_model(args['model'])
print(model.summary())


preds=model.predict(data,batch_size=32).argmax(axis=1)
print(preds)

#display each of the predictions
for (img,pred,label) in zip(data,preds,labels):
    
    label_c=lb.classes_[label.argmax()]
    prediction=lb.classes_[int(pred)]
    cv2.imshow("Prediction: {}, Label: {}".format(prediction,label_c), img)
    cv2.waitKey(0)




