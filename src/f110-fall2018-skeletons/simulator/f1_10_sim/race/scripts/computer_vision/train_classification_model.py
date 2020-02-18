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
import tensorflow.keras.backend as K  
from tensorflow.keras.optimizers import SGD

#import imutils to handle resizing and path importing
from imutils import paths
#import other useful libraries
import matplotlib.pyplot as plt
import numpy as np
import argparse
import imutils
import cv2
import os

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-d", "--dataset", required=True,help="path to input dataset")
ap.add_argument("-o", "--output", required=True,help="directory where we will output the model")
args = vars(ap.parse_args())

# initialize the list of data and labels
data = []
labels = []


#desired height and width these come from the DAVE model
height= 32
width= 32

#count to show the user progress
count=0
# loop over the input images
for imagePath in sorted(list(paths.list_images(args["dataset"]))):
    #load the image, pre-process it, and store it in the data list
    #The directory has the classification label and the image name has the command
    #EXAMPLE: data/{classification}/{time-stamp}~{command}.jpg
    
    #split the path
    split_path=os.path.split(imagePath)
    #get the classification from the path name
    classification= os.path.basename(split_path[0])
    #load the image and normalize
    image = cv2.imread(imagePath)/255.0
    image= imutils.resize(image,width=width)
    #Deterime the padding values for the width and the height to obtain the target dimensions
    #One of these is gonna be zero from above.
    padW=int((width-image.shape[1])/2.0)
    padH=int((width-image.shape[0])/2.0)

    #pad the image then apply one more resizing to handle any rounding issues.
    #There will be cases where we are one pixel off
    #the padding order is top, bottom, left,right
    image=cv2.copyMakeBorder(image,padH,padH,padW,padW,cv2.BORDER_REPLICATE)
    image=cv2.resize(image,(width,height))


    #print(classification,command)
    #image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #image = imutils.resize(image, width=28)
    #image = img_to_array(image)
    #cv2.imshow(classification,image)
    #cv2.waitKey(0)

    #append the image, classification, command
    data.append(image)
    labels.append(classification)
    count+=1
    if(count%500==0):
        print("[INFO] processed {} images".format(count))

data = np.array(data, dtype="float")

labels=np.asarray(labels)


# convert the labels from integers to vectors
#THIS WOULD HAVE SOLVED MY PROBLEM WEEKS AGO
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
model=MiniVGGNet.build(height=height,width=width,depth=3,classes=len(lb.classes_))
#number of epochs
num_epochs=150
opt=SGD(lr=0.03,decay=0.01/num_epochs,momentum=0.9,nesterov=True)
model.compile(loss='categorical_crossentropy',optimizer=opt,metrics=['accuracy'])



#save the best performing models
fname=args['output']
checkpoint = ModelCheckpoint(fname, monitor="val_loss", mode="min",save_best_only=True,save_weights_only=False, verbose=1)

#Let us now instantiate th callbacks
callbacks=[checkpoint]

# train the network
print("[INFO] training network...")
H = model.fit(trainX, trainY, validation_data=(testX, testY),class_weight=classWeight, batch_size=128, callbacks=callbacks , epochs=num_epochs, verbose=1)



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