# import the neccessary package
from keras.utils import np_utils
import numpy as np 
import h5py 

class HDF5DatasetGenerator:

    def __init__(self,dbPath,batchSize,preprocessors=None,
                        aug=None,binarize=True, classes=2):
        # store the batch size, preprocessors, and data augmentor,
        # whether or not the labels should be binarized, along with 
        # the total number of classes
        # aug Defaulting to none, we should also supply a Keras ImageDataGenerator
        # to apply augmentation directly innside our HDF5DatasetGenerator
        # binarize: whether to use binarizers

        self.batchSize= batchSize
        self.preprocessors = preprocessors
        self.aug = aug 
        self.binarize = binarize
        self.classes = classes

        # open the HDF5 database for reading and determine the total 
        # number of entries in the database

        self.db = h5py.File(dbPath)
        self.numImages = self.db["labels"].shape[0]

    # next we need to define a generator function which as the name suggests 
    # is responsible for yielding batchs of images and classes to the .fit_generator
    # function when training a network

    def generator(self,passes=np.inf):
        # initialize the epoch count 

        epochs = 0

        # keep looping infinitely -- the model will stop once we 
        # have reached the desired number of epochs

        while epochs < passes:
            # loop over the HDF5 dataset

            for i in range(0,self.numImages,self.batchSize):
                # extract the images and labels from the HDF5 dataset

                images = self.db["images"][i:i+self.batchSize]
                labels = self.db["labels"][i:i+self.batchSize]

                # check to see if the labels should be binarized 
                if self.binarize:
                    labels = np_utils.to_categorical(labels,self.classes)

                # check to see if our preprocessors are not None
                if self.preprocessors is not None:
                    #initialize the list of processed images
                    procImages =[]

                    # loop over the images 
                    for image in images:
                        # loop over the preprocessors and apply each 
                        # to the image
                        for p in self.preprocessors:
                            image = p.preprocess(image)

                        # update the list of processed images
                        procImages.append(image)

                    # update the images array to the processed images
                    images = np.array(procImages)
                
                # if the data augmentor exists, apply it 
                if self.aug is not None:
                    (images,labels) = next(self.aug.flow(images,labels,batch_size=self.batchSize))
                
                yield (images,labels)
            epochs +=1

    # close the database
    def close(self):
        self.db.close() 