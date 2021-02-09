#import the necessary sklearn packages
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report
import pandas as pd


from fnn import FNN

from keras.callbacks import ModelCheckpoint
from keras.optimizers import SGD
import matplotlib.pyplot as plt
import argparse

# construct the argument parser
ap = argparse.ArgumentParser()
ap.add_argument("-d","--dataset",required=True,help="path to input dataset")
ap.add_argument("-m","--model",required=True,help="path to save the output model")
args = vars(ap.parse_args())

NUM_EPOCHS = 10
BATCH_SIZE = 32


# load model without classifier layers
model = FNN.build((10,1),1,64)

df=pd.read_csv(args['dataset'], sep=',',header=None)

inputs = df[df.columns[:-1]].values
print(inputs.shape)