#!/usr/bin/env python
from __future__ import print_function

# import roslib
import sys
# import rospy
import cv2
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# from std_msgs.msg import Float64
from keras.models import load_model
from image_data import *
import numpy as np
import tensorflow as tf
import keras



class image_converter:

  def __init__(self):

#    self.graph = tf.get_default_graph()
    # self.image_pub = rospy.Publisher("image_topic_2",Image)
    # self.rate = rospy.Rate(20)
    # self.bridge = CvBridge()
    self.timer_to_sending_data = 0
    self.data_save_dir= "/home/nvidia/wecar_ws/src/lane_detection/scripts/saved_angle_data/"
    
    model_path='/home/sreejith/Autonmous_Files/Lane_Detection/line_detection/lane_navigation_check.h5'
  
    self.speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1) 
    self.position = rospy.Publisher('/commands/servo/position', Float64, queue_size=1) 
    
    self.model = load_model(model_path)

    graph = tf.get_default_graph()

    self.speed_value = 1200
    self.position_value = 0.5
    self.speed.publish(0.0)
    self.position.publish(self.position_value)
    #self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.callback()
  # rospy.on_shutdown(self.shutdown)
 

  def img_preprocess(self,image):
    height, _, _ = image.shape
    image = image[int(height/2):,:,:]  # remove top half of the image, as it is not relevant for lane following
    image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)  # Nvidia model said it is best to use YUV color space
    image = cv2.GaussianBlur(image, (3,3), 0)
    image = cv2.resize(image, (200,66)) # input image size (200,66) Nvidia model
    image = image / 255 # normalizing, the processed image becomes black for some reason.  do we need this?
    return image

  def callback(self):
    self.rate.sleep()
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
       print(e)

   # cv2.imshow("Image window", cv_image)
  # cv2.waitKey(1)

    
    #self.speed.publish(self.speed_value)
#    self.position.publish(self.position_value)
    if self.timer_to_sending_data %2 == 0:

      #angle = line_detection(cv_image)
      image= cv2.imread('/home/sreejith/Autonmous_Files/Lane_Detection/line_detection/save_image_angle/_091_-05.png')
      image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
	
      preprocess = self.img_preprocess(image)
      X = np.asarray([preprocess])
      global graph
      with graph.as_default():
        
        steering_angle = self.model.predict(X)
       # keras.backend.clear_session()
       
      print(steering_angle)
      # self.i+=1

      ##- position value 0.4 is left

    #   ##if two lines found and angle is right
    #   if angle == 5:
    #      self.position_value = 0.8
    #      self.speed_value = 900
    #      self.position.publish(self.position_value)
    #      self.speed.publish(self.speed_value)
        
    #       ##if two lines found and angle is left go more left

    #   elif angle == -5: 
    #     self.position_value = -0.2
    #     self.speed_value = 900
    #     self.position.publish(self.position_value)
    #     self.speed.publish(self.speed_value)
        
    #        ##if one line found and angle is left   go slightly left

    #   elif angle == -1:
    #     self.position_value = 0.0
    #     self.speed_value = 900
    #     self.position.publish(self.position_value)
    #     self.speed.publish(self.speed_value)

    #        ##if one line found and angle is right - go slightly right

    #   elif angle == 1:
    #     self.position_value = 0.6
    #     self.speed_value = 900
    #     self.position.publish(self.position_value)
    #     self.speed.publish(self.speed_value)

    #        ##if 2 lines found and angle is zero - position value 0.4 is straight

    #   elif angle==0:
    #     self.position_value = 0.4
    #     self.speed_value = 1200
    #     self.speed.publish(self.speed_value)
    #     self.position.publish(self.position_value)
        
    #   else:
    #     self.position_value = 0.4
    #     self.speed_value = 500
    #     self.speed.publish(self.speed_value)
    #     self.position.publish(self.position_value)

    #   self.timer_to_sending_data = 0


  #   # self.timer_to_sending_data += 1
        
  # def shutdown(self):
  #   self.speed.publish(0)
  #   self.position.publish(0.4)
  #   self.rate.sleep()

def main(args):
  
  rospy.init_node('image_converter', anonymous=True)
  

  ic = image_converter()
    
  #except Exception:
    #ic.shutdown()
   # print("Shutting down")

 # rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

