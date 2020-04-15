#import keras img_to_array, it will order the channels correctly depending on the backend
from tensorflow.python.keras.preprocessing.image import img_to_array


#the constructor for this class accepts an optional parameter named dataFormat. This value defaults to Noe which indicates the setting inside keras.json 
#should be used, we could explicitly set it but its best to let keras set it.

#The benefit of defining a class handle this type of image preprocessing rather than simply calling img_to_array on every single image is that we can now chain
#preprocessors together as we load datasets from disk


class ImageToArrayPreprocessor:
    def __init__(self,dataFormat=None):
        #store the image data format
        self.dataFormat = dataFormat

    def preprocess(self,image):
        #apply the Keras utility function that correctly rearranges the dimensions of the image
        return img_to_array(image,data_format=self.dataFormat)

