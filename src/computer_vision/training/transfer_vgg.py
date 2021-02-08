# import the neccessary packages

# import sys so we can use packages outside of this folder in
# either python 2 or python 3, I know it's janky, chill
import sys
import os
from pathlib import Path 
#insert parent directory into the path
sys.path.insert(0,str(Path(os.path.abspath(__file__)).parent.parent))

from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report
from preprocessing.utils import ImageUtils
import numpy as np
import argparse
import cv2 

# Keras Modules
# example of tending the vgg16 model
from keras.applications.vgg16 import VGG16
from keras.models import Model
from keras.layers import Dense
from keras.layers import Flatten
from keras.applications import imagenet_utils 
from keras.callbacks import ModelCheckpoint
from keras.optimizers import SGD
import matplotlib.pyplot as plt

""" There are perhaps a dozn or more top-performing models for image recognition,
The most popular for transfer learning are: 
- VGG (VGG16)
- GoogLeNet (InceptionV3)
- Residual Networks (ResNet50)

"""
# construct the argument parser
ap = argparse.ArgumentParser()
ap.add_argument("-d","--dataset",required=True,help="path to input dataset")
ap.add_argument("-m","--model",required=True,help="path to save the output model")
args = vars(ap.parse_args())



#desired height, width, epochs
HEIGHT=224
WIDTH= 224
NUM_EPOCHS = 10
BATCH_SIZE = 32



# load model without classifier layers
model = VGG16(include_top=False, input_shape=(HEIGHT, WIDTH, 3),weights='imagenet')

flat1 = Flatten()(model.layers[-1].output)
class1 = Dense(1024, activation='relu')(flat1)
output = Dense(5, activation='softmax')(class1)
# define new model
model = Model(inputs=model.inputs, outputs=output)


# don't train beginning layers where features are rich
for layer in model.layers[:6]:
    print(layer)
    layer.trainable = False

#load the data
iu=ImageUtils()
data,labels=iu.load_from_directory(args['dataset'],HEIGHT,WIDTH,verbose=1)


print("[INFO] Imagenet preprocessing")
training_images=[]
print(data[-1])
#normalize the images
count = 1
for img in data:
    processed_image=imagenet_utils.preprocess_input(np.copy(img))
    training_images.append(processed_image)
    if(count % 100 ==0):
        print("Processed {} Images".format(count))
    count +=1



data=np.asarray(training_images)
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

# compile our model. This needs to be done after our setting our
# layers to being non-trainable

print("[INFO] compiling model...")
#number of epochs
num_epochs=NUM_EPOCHS
opt=SGD(lr=0.00001,decay=0.01/num_epochs,momentum=0.9,nesterov=True)
model.compile(loss='categorical_crossentropy',optimizer=opt,metrics=['accuracy'])

#save the best performing models
fname=args['model']
checkpoint = ModelCheckpoint(fname, monitor="val_loss", mode="min",save_best_only=True,save_weights_only=False, verbose=1)

#Let us now instantiate th callbacks
callbacks=[checkpoint]


print("Fit model on training data")
history = model.fit(
    trainX,
    trainY,
    batch_size=BATCH_SIZE, 
    validation_data=(testX,testY),
    callbacks=callbacks,
    epochs = NUM_EPOCHS
)

# evaluate the network
print("[INFO] evaluating network...")
predictions = model.predict(testX, batch_size=BATCH_SIZE)
print(classification_report(testY.argmax(axis=1),predictions.argmax(axis=1), target_names=lb.classes_))

# plot the training + testing loss and accuracy
plt.style.use("ggplot")
plt.figure()

for i in history.history.keys():
    if i=="loss":
        continue
    plt.plot(np.arange(0, num_epochs), history.history[i], label=i)

plt.title("Training Loss and Accuracy")
plt.xlabel("Epoch #")
plt.ylabel("Loss/Accuracy")
plt.legend()
plt.show()

