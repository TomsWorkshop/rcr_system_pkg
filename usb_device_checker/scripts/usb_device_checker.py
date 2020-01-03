#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)

def status_publisher():
  pub = rospy.Publisher('usb_status', Bool, queue_size=10)
  rospy.init_node('usb_device_checker', anonymous=True)
  r = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    if GPIO.input(12) == GPIO.LOW:
      status = False
    else:
      status = True
    rospy.loginfo(status)
    pub.publish(status)
    r.sleep()

if __name__ == '__main__':
  try:
    status_publisher()
  except rospy.ROSInterruptException: pass