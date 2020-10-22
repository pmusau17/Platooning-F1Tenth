# import model averaging class
from combine_models import CombineModels
import argparse
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split


# import sys so we can use packages outside of this folder in
# either python 2 or python 3, I know it's janky, chill
import sys
import os
from pathlib import Path 

#insert parent directory into the path
sys.path.insert(0,str(Path(os.path.abspath(__file__)).parent.parent))

# import the preprocessing helper classes to load and create the data
from preprocessing.utils import ImageUtils


# define the argument parser
ap = argparse.ArgumentParser()
ap.add_argument("-m", "--models", required=True,help="Path to input models")
ap.add_argument("-s", "--search", required=True,help="Search Pattern That Selects Desired Models (eg. *.hdf5, minivgg*.py)")
ap.add_argument("-d", "--dataset", required=True,help="Path to the dataset you want to test this average model from")
args = vars(ap.parse_args())

# Load all the models by passing them to the combine models constructor
cm= CombineModels(args['models'],args['search'])

# get the number of models if you want to do a simple average
num_models = len(cm.models)
weights = [(1)/float(num_models) for i in range(0,num_models)]

# create a model by averaging the weights using the weights
# above 
model=cm.model_weight_ensemble(weights,plot_layer_weights=True)
print(model.summary())

# get the height and width from the model without averaging

HEIGHT = model.layers[0].input_shape[1]
WIDTH = model.layers[0].input_shape[2]

#load the image utils
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

_, acc = model.evaluate(data,labels)
print(acc)
