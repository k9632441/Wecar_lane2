#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64

from wecar import *

def run():
    rospy.init_node('line_detect', anonymous=True)
    run_car = LineCar()
    run_car.set_cam(0)

    speed = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1) 
    position = rospy.Publisher('/commands/servo/position', Float64, queue_size=1) 

    while not rospy.is_shutdown():
        steer = next(run_car.detect_line())
        if not steer == -99:
            position_value = ((steer * np.pi) / float(180)) + 0.4501
            speed_value = 2400 - abs((steer) * 5)
        else:
            position_value = 0.4501
            speed_value = 2400	
        print("{0} calced_steer, {1} speed, {2} steer".format(position_value, speed_value, steer))
        position.publish(position_value)
        speed.publish(speed_value)

if __name__ == '__main__':
    run()

