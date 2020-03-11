#import the necessary sklearn packages
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report

#import nvidias dave2 architechture
from nn.conv.dave2 import DAVE2

#import tensorflow libraries
from tensorflow.python.keras.preprocessing.image import img_to_array
from tensorflow.python.keras.utils import np_utils
from tensorflow.python.keras.callbacks import ModelCheckpoint
import tensorflow.keras.backend as K  
from tensorflow.keras.optimizers import SGD
from tensorflow.keras.optimizers import Adam

#define a custom metric for DAEV, accuracy doesn't cut it
def customAccuracy(y_true, y_pred):
    diff = K.abs(y_true-y_pred) #absolute difference between correct and predicted values
    correct = K.less(diff,0.01) #tensor with 0 for false values and 1 for true values
    return K.mean(correct) #sum all 1's and divide by the total. 

#import imutils to handle resizing and path importing
from imutils import paths
#import other useful libraries
import matplotlib.pyplot as plt
import numpy as np
import argparse
import imutils
import cv2
import os
from preprocessing.utils import ImageUtils

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dataset", required=True,help="path to input dataset")
ap.add_argument("-o", "--output", required=True,help="directory where we will output the model")
args = vars(ap.parse_args())

# initialize the list of data and labels
data = []
commands= []

#desired height and width these come from the DAVE model
height=66
width= 200

#load the image utils
iu=ImageUtils()

data,labels=iu.load_from_directory(args['dataset'],height,width,verbose=1,regression=True)


#normalize the images and commands
commands=labels/0.6108652353
data=data/255.0


#End to End data
(EndtrainX, EndtestX, EndtrainY, EndtestY) = train_test_split(data,commands, test_size=0.05, random_state=42)
# initialize the model
print("[INFO] compiling model...")

#number of epochs
num_epochs=1
#decay=0.01/num_epochs
opt=SGD(lr=0.05,decay=0.01/num_epochs,momentum=0.9,nesterov=True)
#opt=Adam(learning_rate=0.1, beta_1=0.9, beta_2=0.999, amsgrad=False)


model = DAVE2.build(height=66, width=200, depth=3, classes=1)   
print(model.summary())
model.compile(loss="mean_squared_error", optimizer=opt,metrics=[customAccuracy])

#model.compile(loss="mean_absolute_percentage_error", optimizer=opt,metrics=[customAccuracy])


#save the best performing models
fname=args['output']
#checkpoint = ModelCheckpoint(fname, monitor="val_loss", mode="min",save_best_only=True,save_weights_only=False, verbose=1)
checkpoint = ModelCheckpoint(fname,monitor="customAccuracy", mode="max",save_best_only=True,save_weights_only=False, verbose=1)

#Let us now instantiate th callbacks
callbacks=[checkpoint]
# train the network
print("[INFO] training network...")
H = model.fit(EndtrainX, EndtrainY, validation_data=(EndtestX, EndtestY), batch_size=256, callbacks=callbacks , epochs=num_epochs, verbose=1)


# evaluate the network
print("[INFO] evaluating network...")
predictions = model.predict(EndtestX, batch_size=64)

print(predictions[:10],EndtestY[:10])
print(max(predictions))

# plot the training + testing loss and accuracy
plt.style.use("ggplot")
plt.figure()

for i in H.history.keys():
    plt.plot(np.arange(0, num_epochs), H.history[i], label=i)

plt.title("Training Loss and Accuracy")
plt.xlabel("Epoch #")
plt.ylabel("Loss/Accuracy")
plt.legend()
plt.show()