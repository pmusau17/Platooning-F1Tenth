#import the necessary sklearn packages
from sklearn.preprocessing import LabelBinarizer
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report
import pandas as pd



from fnn import FNN

from keras.callbacks import ModelCheckpoint
from keras.optimizers import SGD
import matplotlib.pyplot as plt
import keras.backend as K
import argparse
from keras.losses import mean_squared_error
import numpy as np  

# construct the argument parser
ap = argparse.ArgumentParser()
ap.add_argument("-d","--dataset",required=True,help="path to input dataset")
ap.add_argument("-m","--model",required=True,help="path to save the output model")
args = vars(ap.parse_args())

NUM_EPOCHS = 100
BATCH_SIZE = 3791


def soft_acc(y_true, y_pred):
    return K.mean(K.equal(K.round(y_true), K.round(y_pred)))

# load model without classifier layers
model = FNN.build(9,1,64)

df=pd.read_csv(args['dataset'], sep=',',header=None).dropna()

inputs = df[df.columns[0:9]].values

#inputs = inputs.reshape((inputs.shape[0],inputs.shape[1],1))
outputs = df[df.columns[9]].values


# partition the data into training and testing splits using 80% of
# the data for training and the remaining 20% for testing
#classification data
(trainX, testX, trainY, testY) = train_test_split(inputs,outputs, test_size=0.20,random_state=15)

#print(trainX[0])
#print(trainY[0])
#exit()


print("[INFO] compiling model...")
#number of epochs
num_epochs=NUM_EPOCHS
opt=SGD(lr=0.05,decay=0.02/num_epochs,momentum=0.9,nesterov=True)
model.compile(loss=mean_squared_error,optimizer=opt,metrics=['mae','mse','mape'])

#save the best performing models
fname=args['model']
checkpoint = ModelCheckpoint(fname, monitor="mae", mode="min",save_best_only=True,save_weights_only=False, verbose=1)

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

mse = np.mean((predictions-testY)**2)
print("test mse:",mse)

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