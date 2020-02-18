#import the necessary sklearn packages
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report

#import nvidias dave2 architechture
from nn.conv.dave2 import DAVE2

#import tensorflow libraries
from tensorflow.python.keras.preprocessing.image import img_to_array
from tensorflow.python.keras.utils import np_utils
from tensorflow.python.keras.callbacks import ModelCheckpoint
import tensorflow.keras.backend as K  
from tensorflow.keras.optimizers import SGD

#define a custom metric for DAEV, accuracy doesn't cut it
def customAccuracy(y_true, y_pred):
    diff = K.abs(y_true-y_pred) #absolute difference between correct and predicted values
    correct = K.less(diff,0.01) #tensor with 0 for false values and 1 for true values
    return K.mean(correct) #sum all 1's and divide by the total. 

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
commands= []

#desired height and width these come from the DAVE model
height=66
width= 200

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
    #get the command
    command=split_path[1].replace('.jpg','').split('~')[1:]
    #convert it back to the command
    command=float('.'.join(command))
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

    #append the image, classification, command
    data.append(image)
    commands.append(command)
    count+=1
    if(count%500==0):
        print("[INFO] processed {} images".format(count))

data = np.array(data, dtype="float")
#normalize the commands
commands=np.asarray(commands, dtype="float")/0.6108652353
print(commands.min(axis=0),commands.max(axis=0))




#End to End data
(EndtrainX, EndtestX, EndtrainY, EndtestY) = train_test_split(data,commands, test_size=0.20, random_state=42)
# initialize the model
print("[INFO] compiling model...")
#number of epochs
num_epochs=8000
#decay=0.01/num_epochs
opt=SGD(lr=0.01,momentum=0.9,nesterov=True)
model = DAVE2.build(height=66, width=200, depth=3, classes=1)
model.compile(loss="mean_squared_error", optimizer=opt,metrics=[customAccuracy])

#save the best performing models
fname=args['output']
checkpoint = ModelCheckpoint(fname, monitor="val_loss", mode="min",save_best_only=True,save_weights_only=False, verbose=1)

#Let us now instantiate th callbacks


# train the network
print("[INFO] training network...")


callbacks=[checkpoint]
H = model.fit(EndtrainX, EndtrainY, validation_data=(EndtestX, EndtestY), batch_size=128, callbacks=callbacks , epochs=num_epochs, verbose=1)


# evaluate the network
print("[INFO] evaluating network...")
predictions = model.predict(EndtestX, batch_size=64)

print(predictions[:10],EndtestY[:10])

# plot the training + testing loss and accuracy
plt.style.use("ggplot")
plt.figure()

for i in H.history.keys():
    plt.plot(np.arange(0, num_epochs), H.history[i], label=i)

plt.title("Training Loss and Accuracy")
plt.xlabel("Epoch #")
plt.ylabel("Loss/Accuracy")
plt.legend()
plt.show()