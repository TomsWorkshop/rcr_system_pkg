#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import pyudev

context = pyudev.Context()
monitor = pyudev.Monitor.from_netlink(context)
monitor.filter_by(subsystem='usb')
monitor.start()

status = False

def checker():
  for device in iter(monitor.poll, None):
    if device.action == 'add' or device.action == 'bind':
      status = True
      print('USB device status : connected')
    elif device.action == 'remove' or device.action == 'unbind':
      status = False
      print('USB device status : disconnected')

def status_pub():
  pub = rospy.Publisher('usb_status', Bool, queue_size=10)
  rospy.init_node('usb_device_checker', anonymous=True)
  r = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    checker()
    rospy.loginfo(status)
    pub.publish(status)
    r.sleep()

if __name__ == '__main__':
  try:
    status_pub()
  except rospy.ROSInterruptException: pass