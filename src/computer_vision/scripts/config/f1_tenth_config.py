# This is our Python Based Configuration File
IMAGES_PATH = "../data/"


# define the path to the output training, validation, and testing HDF5 files
TRAIN_HDF5 = "../data/hdf5/train.hdf5"
VAL_HDF5 = "../data/hdf5/val.hdf5"
TEST_HDF5 = "../data/hdf5/test.hdf5"

# define the path to the dataset mean
# The mean will be used to store the average red, green, blue ouxel intensity across 
# the tneire (training) dataset
DATASET_MEAN = "../data/means/f1tenth_mean.json"