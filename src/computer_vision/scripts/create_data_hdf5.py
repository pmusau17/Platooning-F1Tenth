# import sys so we can use packages outside of this folder in
# either python 2 or python 3
import sys
import os
from pathlib import Path 
#insert parent directory into the path
sys.path.insert(0,str(Path(os.path.abspath(__file__)).parent.parent))

from input_output.hdf5datasetwriter import HDF5DatasetWriter
import config.f1_tenth_config as config
from preprocessing.utils import ImageUtils
from preprocessing.aspectawarepreprocessor import AspectAwarePreprocessor

from sklearn.preprocessing import LabelEncoder
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
import numpy as np 
import progressbar
import json
import cv2
import os 

""" This script will create an hdf5 file. This will allow one to train models on data that is 
too large to be held in main memory at one time"""


# Instantiate Image Utils for easy loading of data

iu = ImageUtils()
paths,labels= iu.load_imagepaths_and_labels(config.IMAGES_PATH,verbose=1)


# create the label encoder
le = LabelEncoder()
labels = le.fit_transform(labels)

# shuffle the data 

paths, labels = shuffle(paths, labels)

# construct a list pairing their training, validation, and testing
# image paths along with their corresponding labels and output HDF5
# files

datasets = [
    ("train",paths,labels,config.TRAIN_HDF5)#,
    #("val",valPaths,valLabels,config.VAL_HDF5),
    #("test",testPaths,testLabels,config.TEST_HDF5)
]

# intialize the image preprocessor and list of RGB channel averages
# 72 comes from the ratio that AlexNet used which was 256/227
# My target size is 64 so 256/227 * 64 = 72

HEIGHT = 72
WIDTH = 72

aap = AspectAwarePreprocessor(HEIGHT,WIDTH)
(R,G,B) = ([],[],[])

# loop over the dataset tuples
for (dType, paths, labels, outputPath) in datasets:
    # create HDF5 writer
    print("[INFO] building {}...".format(outputPath))
    writer = HDF5DatasetWriter((len(paths),HEIGHT,WIDTH,3),outputPath)


    # Initialize the progress bar

    widgets = ["Building Dataset: ",progressbar.Percentage(),' ',progressbar.Bar(),' ',progressbar.ETA()]
    pbar = progressbar.ProgressBar(maxval=len(paths),widgets=widgets).start()

    # loop over the image paths
    for (i,(path,label)) in enumerate(zip(paths,labels)):
        # load the image and process it

        image = cv2.imread(path)
        image = aap.preprocess(image)

        # if we are building the training dataset, then compute the
        # mean of each channel in the image, then update the 
        # respective lists

        if dType == "train":
            (b,g,r) = cv2.mean(image)[:3]
            R.append(r)
            G.append(g)
            B.append(b)

        # add the image and label to the HDF5 dataset
        writer.add([image],[label])
        pbar.update(i)

    pbar.finish
    writer.close()

    # construct the dictionary of averages, then serialize the means to a JSON file

    print("[INFO] serializing means...")
    
    D = {"R": np.mean(R), "G": np.mean(G),"B":np.mean(B)}
    
    f = open(config.DATASET_MEAN,'w')
    f.write(json.dumps(D))
    f.close()
