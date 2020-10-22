# import packages
import h5py
import os

""" This calls will be responsible for taking an input set of
Numpy Arrays and writing them to an HDF5 format"""

class HDF5DatasetWriter:

    def __init__(self, dims,outputPath,dataKey="images",buffSize=1000):
        # Check to see if the output path exists and if so raise an exception
        if(os.path.exists(outputPath)):
            raise ValueError("The Supplied \'outputPath\' already exists "
                            "and cannot be overwritten. Manually delete "
                            "the file before continuing",outputPath)
        
        # open the HDF5 database for writing and create two datasets:
        # one to store the images/features and another to store the class
        # labels

        self.db = h5py.File(outputPath,'w')
        # create the dataset for images
        self.data = self.db.create_dataset(dataKey,dims,dtype="float")
        # create the labels of shape (N,)
        self.labels = self.db.create_dataset("labels",(dims[0],),dtype='int')

        # store the buffer size then initialize the buffer itself
        # along with the index into the datasets

        self.buffSize =buffSize
        self.buffer = {"data": [],"labels":[]}
        self.idx =0

    def add(self, rows,labels):
        # add the rows and labels to the buffer
        # pytho list extend() extends the list by adding all items of a lit (passed as an argument)
        # to the end
        self.buffer['data'].extend(rows)
        self.buffer['labels'].extend(labels)
        #check to see if the buffer needs to be flushed to disk
        if len(self.buffer['data']) >=self.buffSize:
            self.flush()
    
    # flushes data to the disk once it reaches the buffersize
    def flush(self):
        # write the rows to disk and then reset the buffer
        i = self.idx + len(self.buffer['data'])
        self.data[self.idx:i] = self.buffer['data']
        self.labels[self.idx:i] = self.buffer["labels"]
        self.idx =i
        self.buffer = {"data":[],"labels":[]}

    # utility function named storeClassLabels which, if called, will store
    # the raw string names of the class labels in a seperate dataset

    def storeClassLabels(self,classLabels):
        # create a dataset to store the actual class label names
        # then store the class labels

        #apparently this thing is so picky it has its own datatype    
        dt = h5py.special_dtype(vlen=unicode)

        labelSet = self.db.create_dataset("label_names",(len(classLabels),),dtype=dt)
        labelSet[:] = classLabels

    def close(self):
        # check to see if there are any other entries in the buffer
        # that nee to be flushed to disk

        if len(self.buffer["data"]) > 0:
            self.flush()

        # close the dataset
        self.db.close() 


