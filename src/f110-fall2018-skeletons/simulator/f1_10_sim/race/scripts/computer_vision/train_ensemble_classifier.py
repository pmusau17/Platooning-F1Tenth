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

# desired height, width, epochs
HEIGHT= 64
WIDTH= 64
NUM_EPOCHS = 100
BATCH_SIZE = 128

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dataset", required=True,help="path to input dataset")
ap.add_argument("-o", "--output", required=True,help="directory where we will output the model")
ap.add_argument("-l", "--logs", required=True,help="directory where we will output the plots and accuracy reports")
ap.add_argument("-n","--num-models",type=int,default=5,help="# of models to train")
args = vars(ap.parse_args())

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

# Data Augmentation 
# Allow images to be randomly brightened, rescaled, shifted by a factor of 0.1
# and randomly horizontally flipped

# define the image augmentation generator
aug= ImageDataGenerator(rotation_range=5, brightness_range=[0.5,1.5], zoom_range=[0.9,1.1],rescale=1.0/255.0,fill_mode="nearest")

# loop over the number of models to train

batch_size=BATCH_SIZE
num_epochs=NUM_EPOCHS
for i in np.arange(0, args['num_models']):

   # build the model 
   model=MiniVGGNet.build(height=HEIGHT,width=WIDTH,depth=3,classes=len(lb.classes_))

   # define the optimizer 
   opt=SGD(lr=0.05,decay=0.01/num_epochs,momentum=0.9,nesterov=True)

   # compile the model
   model.compile(loss='categorical_crossentropy',optimizer=opt,metrics=['accuracy']) 

   # define the filename 
   p = [args['output'],"model_{}.hdf5".format(i)]
   fname=os.path.sep.join(p)

   checkpoint = ModelCheckpoint(fname, monitor="val_loss", mode="min",save_best_only=True,save_weights_only=False, verbose=1)

   #Let us now instantiate th callbacks
   callbacks=[checkpoint]

   H = model.fit_generator(aug.flow(trainX,trainY,batch_size=batch_size), 
                            validation_data=(testX,testY), 
                            steps_per_epoch=len(trainX)//batch_size,
                            epochs =num_epochs,verbose=1,
                            callbacks=callbacks)
   # Evaluate the network
   predictions = model.predict(testX,batch_size=BATCH_SIZE)
   report =classification_report(testY.argmax(axis=1),
                                predictions.argmax(axis=1),
                                target_names=lb.classes_)

   # save the classification report to file
   p = [args['logs'],"model_{}_report.txt".format(i)]
   f = open(os.path.sep.join(p),"w")
   f.write(report)
   f.close()

   # plot the training loss and accuracy

   p = [args['output'],"model_{}.png".format(i)]
   plt.style.use("ggplot")
   plt.figure()
   
   for key in H.history.keys():
       plt.plot(np.arange(0, NUM_EPOCHS), H.history[key], label=key)
       
   plt.title("Training Loss and Accuracy for model {}".format(i))
   plt.xlabel("Epoch #")
   plt.ylabel("Loss/Accuracy")
   plt.legend() 
   plt.savefig(os.path.sep.join(p))
   plt.close()    
