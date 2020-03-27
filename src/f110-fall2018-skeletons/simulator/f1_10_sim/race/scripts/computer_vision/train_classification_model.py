#import the necessary sklearn packages
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report

#import minivggnet, shallownet
from nn.conv.minivggnet import MiniVGGNet
from nn.conv.shallownet import ShallowNet

#import tensorflow libraries
from tensorflow.python.keras.preprocessing.image import img_to_array
from tensorflow.python.keras.utils import np_utils
from tensorflow.python.keras.callbacks import ModelCheckpoint
from tensorflow.python.keras.preprocessing.image import ImageDataGenerator
import tensorflow.keras.backend as K  
from tensorflow.keras.optimizers import SGD

#import imutils to handle resizing and path importing
from imutils import paths
#import other useful libraries
import matplotlib.pyplot as plt
import numpy as np
import argparse
from preprocessing.utils import ImageUtils
import imutils
import cv2
import os

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dataset", required=True,help="path to input dataset")
ap.add_argument("-o", "--output", required=True,help="directory where we will output the model")
args = vars(ap.parse_args())

#desired height, width, epochs
HEIGHT= 64
WIDTH= 64
NUM_EPOCHS = 100
BATCH_SIZE = 128

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

# initialize the model
print("[INFO] compiling model...")
model=MiniVGGNet.build(height=HEIGHT,width=WIDTH,depth=3,classes=len(lb.classes_))
#number of epochs
num_epochs=NUM_EPOCHS
opt=SGD(lr=0.05,decay=0.01/num_epochs,momentum=0.9,nesterov=True)
model.compile(loss='categorical_crossentropy',optimizer=opt,metrics=['accuracy'])

#save the best performing models
fname=args['output']
checkpoint = ModelCheckpoint(fname, monitor="val_loss", mode="min",save_best_only=True,save_weights_only=False, verbose=1)

#Let us now instantiate th callbacks
callbacks=[checkpoint]


# define the image augmentation generator
aug= ImageDataGenerator(rotation_range=5, brightness_range=[0.5,1.5], zoom_range=[0.7,1.3],rescale=1.0/255.0,fill_mode="nearest")


# train the network
print("[INFO] training network...")

#the batch size you are using
batch_size=BATCH_SIZE
H = model.fit_generator(aug.flow(trainX,trainY,batch_size=batch_size), validation_data=(testX,testY), steps_per_epoch=len(trainX)//batch_size,epochs =num_epochs,verbose=1,callbacks=callbacks)

# evaluate the network
print("[INFO] evaluating network...")
predictions = model.predict(testX, batch_size=64)
print(classification_report(testY.argmax(axis=1),predictions.argmax(axis=1), target_names=lb.classes_))

# plot the training + testing loss and accuracy
plt.style.use("ggplot")
plt.figure()

for i in H.history.keys():
    if i=="loss":
        continue
    plt.plot(np.arange(0, num_epochs), H.history[i], label=i)

plt.title("Training Loss and Accuracy")
plt.xlabel("Epoch #")
plt.ylabel("Loss/Accuracy")
plt.legend()
plt.show()