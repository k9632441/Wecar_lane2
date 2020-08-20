#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64

from image_data import *
class image_converter:

  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.rate = rospy.Rate(20)
    self.bridge = CvBridge()
    self.timer_to_sending_data = 0
    self.data_save_dir= "/home/nvidia/wecar_ws/src/lane_detection/scripts/saved_angle_data/"
	
    self.speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1) 
    self.position = rospy.Publisher('/commands/servo/position', Float64, queue_size=1) 

    self.speed_value = 1200
    self.position_value = 0.5
   # self.speed.publish(0.0)
    self.position.publish(self.position_value)
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    rospy.on_shutdown(self.shutdown)
 

    self.i = 0

  def callback(self,data):
    self.rate.sleep()
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
  #  cv2.imshow("Image window", cv_image)
   # cv2.waitKey(1)

    #self.speed.publish(self.speed_value)
#    self.position.publish(self.position_value)
    if self.timer_to_sending_data %2 == 0:

      angle = line_detection(cv_image)
      cv2.imwrite("%s%03d_%04d.png" % (self.data_save_dir,self.i,angle),cv_image)
      print(angle)
      self.i+=1
      
      ##- position value 0.4 is left

      ##if two lines found and angle is right
      if angle == 5:
	 self.position_value = 0.8
         self.speed_value = 900
	 self.position.publish(self.position_value)
         self.speed.publish(self.speed_value)
	
    ##if two lines found and angle is left go more left

      elif angle == -5: 
	self.position_value = -0.2
        self.speed_value = 900
	self.position.publish(self.position_value)
	self.speed.publish(self.speed_value)
	
     ##if one line found and angle is left   go slightly left

      elif angle == -1:
	self.position_value = 0.0
        self.speed_value = 900
	self.position.publish(self.position_value)
	self.speed.publish(self.speed_value)

     ##if one line found and angle is right - go slightly right

      elif angle == 1:
	self.position_value = 0.6
        self.speed_value = 900
	self.position.publish(self.position_value)
	self.speed.publish(self.speed_value)

     ##if 2 lines found and angle is zero - position value 0.4 is straight

      elif angle==0:
        self.position_value = 0.4
	self.speed_value = 1200
	self.speed.publish(self.speed_value)
        self.position.publish(self.position_value)
	
      else:
	self.position_value = 0.4
	self.speed_value = 500
	self.speed.publish(self.speed_value)
	self.position.publish(self.position_value)

      self.timer_to_sending_data = 0


    self.timer_to_sending_data += 1
    
  def shutdown(self):
    self.speed.publish(0)
    self.position.publish(0.4)
    self.rate.sleep()

def main(args):
  

  rospy.init_node('image_converter', anonymous=True)
  

  ic = image_converter()
    
  #except Exception:
    #ic.shutdown()
   # print("Shutting down")

  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

