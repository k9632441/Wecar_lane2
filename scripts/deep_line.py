#!/usr/bin/env python
from __future__ import print_function
import sys
import cv2
from keras.models import load_model

import numpy as np
import tensorflow as tf
import keras
model_path='/home/nvidia/wecar_ws/src/lane_detection/scripts/lane_navigation_final.h5'
model = load_model(model_path)
def img_preprocess(image):
  height, _, _ = image.shape
  image = image[int(height/2):,:,:]  # remove top half of the image, as it is not relevant for lane following
  image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)  # Nvidia model said it is best to use YUV color space
  image = cv2.GaussianBlur(image, (3,3), 0)
  image = cv2.resize(image, (200,66)) # input image size (200,66) Nvidia model
  image = image / 255 # normalizing, the processed image becomes black for some reason.  do we need this?
  return image

def dl_angle(image):
    print("loading the model")

   # image= cv2.imread(image)
    print("model is loaded")
    image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)

    preprocess = img_preprocess(image)
    X = np.asarray([preprocess])
      
    steering_angle = model.predict(X)[0]
     
    return steering_angle
 

