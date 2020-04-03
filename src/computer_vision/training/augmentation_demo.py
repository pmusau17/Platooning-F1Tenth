#import the neccessary packages
from tensorflow.python.keras.preprocessing.image import ImageDataGenerator
from tensorflow.python.keras.preprocessing.image import img_to_array
from tensorflow.python.keras.preprocessing.image import load_img

# import sys so we can use packages outside of this folder in
# either python 2 or python 3
import sys
#insert parent directory into the path
sys.path.insert(0,'..')

from preprocessing.utils import ImageUtils
import numpy as np 
import argparse
import cv2


ap = argparse.ArgumentParser()
ap.add_argument("-i","--image",required=True,help="path to the input image")
ap.add_argument("-o", "--output", required=False, help="path to output directory to store augmentation examples")
ap.add_argument("-p","--prefix",type=str,default="image",help="output filename prefix")

args= vars(ap.parse_args())

#load the input image, convert it to a Numpy array, and then reshape it to have an extra dimension
print("[INFO] loading example image...")
image = cv2.imread(args["image"]).astype("float")/255.0
print(image)
cv2.imshow("original",image)
cv2.waitKey(0)
image = np.expand_dims(image,axis=0)

#contruct the image generator for data augmentation
#rescaling makes sure that the image is back to 1/255
aug= ImageDataGenerator(rotation_range=5, brightness_range=[0.5,1.5], zoom_range=[0.7,1.3],rescale=1.0/255.0,fill_mode="nearest")
total = 0

# construt the actual Python generator
print ("[INFO] generating images...")

images=[]

# You can save the images to fie effectively creating synthetic data 
if(args['output']):
    imageGen=aug.flow(image,save_to_dir=args["output"],save_prefix=args["prefix"],save_format="jpg",batch_size=1)
else:
    imageGen=aug.flow(image,batch_size=1)

#loop over examples from the image data augmentation generator
for image in imageGen:
    # increment the counter
    total +=1
    images.append(image[0])
    print(image.shape)

    # if we have reached 10 examples, break from the loop
    if total == 100:
        break

# visualize the image
for i in images:
    cv2.imshow(args["image"],i)
    cv2.waitKey(40)