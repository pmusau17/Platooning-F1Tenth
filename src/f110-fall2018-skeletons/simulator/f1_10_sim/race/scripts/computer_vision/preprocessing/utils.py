#import the necessary packages
from imutils import paths
import os
import cv2
import imutils
import numpy as np

"""Class for loading images from a directory, probably more later.
    Assumes data is structured in the following format: data/{classification}/{time-stamp}~{command}.jpg
"""

class ImageUtils:
    def __init__(self):
        #image data,count and labels
        self.data=[]
        self.labels=[]
        self.count=0

    def load_from_directory(self,dataset,height,width,verbose=0):
        #count to show the user progress
        count=0
        # loop over the input images
        for imagePath in sorted(list(paths.list_images(dataset))):
            #load the image, pre-process it, and store it in the data list
            #The directory has the classification label and the image name has the command
            #EXAMPLE: data/{classification}/{time-stamp}~{command}.jpg
            
            #split the path
            split_path=os.path.split(imagePath)
            #get the classification from the path name
            classification= os.path.basename(split_path[0])
            #load the image and normalize
            image = cv2.imread(imagePath)
            image= imutils.resize(image,width=width)
            #Determine the padding values for the width and the height to obtain the target dimensions
            #One of these is gonna be zero from above.
            padW=int((width-image.shape[1])/2.0)
            padH=int((width-image.shape[0])/2.0)

            #pad the image then apply one more resizing to handle any rounding issues.
            #There will be cases where we are one pixel off
            #the padding order is top, bottom, left,right
            image=cv2.copyMakeBorder(image,padH,padH,padW,padW,cv2.BORDER_REPLICATE)
            image=cv2.resize(image,(width,height))

            #append the image, classification, command
            self.data.append(image)
            self.labels.append(classification)
            count+=1
            if(count%500==0 and verbose):
                print("[INFO] processed {} images".format(count))

        self.data = np.array(self.data, dtype="float")
        self.labels=np.asarray(self.labels)
        return self.data,self.labels

    def reshape_image(self,image,height,width):
        image= imutils.resize(image,width=width)
        #Determine the padding values for the width and the height to obtain the target dimensions
        #One of these is gonna be zero from above.
        padW=int((width-image.shape[1])/2.0)
        padH=int((width-image.shape[0])/2.0)

        #pad the image then apply one more resizing to handle any rounding issues.
        #There will be cases where we are one pixel off
        #the padding order is top, bottom, left,right
        image=cv2.copyMakeBorder(image,padH,padH,padW,padW,cv2.BORDER_REPLICATE)
        image=cv2.resize(image,(width,height))
        return image

    