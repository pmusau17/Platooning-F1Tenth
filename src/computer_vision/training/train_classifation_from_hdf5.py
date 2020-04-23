
# import sys so we can use packages outside of this folder in
# either python 2 or python 3
import sys
import os
from pathlib import Path 
#insert parent directory into the path
sys.path.insert(0,str(Path(os.path.abspath(__file__)).parent.parent))

# import the neccessary packages
from sklearn.metrics import classification_report
from sklearn.metrics import accuracy_score
import argparse
import h5py 
import json 

from tensorflow.python.keras.callbacks import ModelCheckpoint
from tensorflow.python.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.optimizers import SGD
from preprocessing.meanpreprocessor import MeanPreprocessor
from preprocessing.patchpreprocessor import PatchPreprocessor
from preprocessing.simplepreprocessor import SimplePreprocessor
from preprocessing.imagetoarraypreprocessor import ImageToArrayPreprocessor
from input_output.hdf5datasetgenerator import HDF5DatasetGenerator

# import network models
from nn.conv.shallownet import ShallowNet
from nn.conv.minivggnet import MiniVGGNet
import cv2
import numpy as np 

# set the matplotlib backend so figures can be saved in the background
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt 


# Define constants 

HEIGHT = 64
WIDTH = 64
BATCH_SIZE = 128
CLASSES = 5 
NUM_EPOCHS = 100

# construct the argument parser and parse the arguments 
ap = argparse.ArgumentParser()
ap.add_argument('-d','--db',required=True,help="path to HDF5 database")
ap.add_argument('-m','--model',required=True,help="path to output model")
ap.add_argument('-j','--json',required=True,help="path to json file with means")
args=vars(ap.parse_args())


# load the RGB means for the training set
means = json.loads((open(args['json']).read()))

# define the image augmentation generator
aug= ImageDataGenerator(rotation_range=2, brightness_range=[0.5,1.5], zoom_range=[0.9,1.1],fill_mode="nearest")

# initialize the image preprocesors 
sp = SimplePreprocessor(HEIGHT,WIDTH)
pp = PatchPreprocessor(HEIGHT,WIDTH)

mp = MeanPreprocessor(means['R'],means['G'],means['B'])
iap = ImageToArrayPreprocessor()

# initialize the training and validation dataset generators
trainGen = HDF5DatasetGenerator(args['db'],BATCH_SIZE,aug=aug,preprocessors=[pp,mp,iap],classes=CLASSES)

# uncomment the following to visualize an image 
#(images,labels) = next(trainGen.generator())
#cv2.imshow("{}".format(trainGen.db['label_names'][labels[0].argmax()]),images[0]/255.0)
#cv2.waitKey(0)

# initialize the model
print("[INFO] compiling model...")
model=MiniVGGNet.build(height=HEIGHT,width=WIDTH,depth=3,classes=CLASSES)
#number of epochs
num_epochs=NUM_EPOCHS
opt=SGD(lr=0.05,momentum=0.9,nesterov=True)
model.compile(loss='categorical_crossentropy',optimizer=opt,metrics=['accuracy'])

#save the best performing models
fname=args['model']
checkpoint = ModelCheckpoint(fname, monitor="loss", mode="min",save_best_only=True,save_weights_only=False, verbose=1)
callbacks =[checkpoint]

H = model.fit_generator(
    trainGen.generator(),
    steps_per_epoch = trainGen.numImages // BATCH_SIZE,
    max_queue_size = BATCH_SIZE * 2,
    epochs= NUM_EPOCHS,
    callbacks = callbacks, verbose =1 
) 

# plot and save figure 
plt.style.use("ggplot")
plt.figure()
   
for key in H.history.keys():
    plt.plot(np.arange(0, NUM_EPOCHS), H.history[key], label=key)
       
plt.title("Training Loss and Accuracy")
plt.xlabel("Epoch #")
plt.ylabel("Loss/Accuracy")
plt.legend() 
plt.savefig('../plots/model.png')
plt.close()    

# close the HDF5 datasets
trainGen.close()