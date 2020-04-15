#import packages
import imutils
import cv2

class AspectAwarePreprocessor:
    #constructor for preprocessor
    def __init__(self,width,height,inter=cv2.INTER_AREA):
        # store the target image width, height, and interpolation
        # method used when resizing
        self.width = width
        self.height = height
        self.inter = inter 

    def preprocess(self,image):
        # grab the dimensions of the image and then initialize
        # the deltas to use when cropping
        
        (h,w)=image.shape[:2]

        # These parameters are the delta offsets that will be used to crop the image
        # along the larger dimension
        dW =0
        dH =0 

        # if the width is smaller than the height, then resize along the width (i.e., the smaller dimension)
        # then update the deltas to crop the height to the deisred dimension

        if w < h: 
            image = imutils.resize(image, width=self.width,inter=self.inter)

            dH= int((image.shape[0]-self.height)/2.0) # for my intuitive understanding this is the difference
            #in the desired height and what the crop did
        else:
            image = imutils.resize(image,height=self.height,inter=self.inter)

            dW= int((image.shape[1]-self.width)/2.0)

        # Now that the image has been resized you need to re-grab the width and height followed
        # by performing the crop of the center of the image

        (h,w)=image.shape[:2] 
        image=image[dH:h-dH, dW:w-dW]

        # finally, resize the image to the provided spatial dimensions to ensure our output
        # image is always a fixed size

        return cv2.resize(image,(self.width,self.height),interpolation=self.inter)

