#!/usr/bin/env python

import rospy
import smbus            # use I2C
import math             # mathmatics
from time import sleep  # time module
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

class mpu6050():
    # slave address
    DEV_ADDR    = 0x68         # device address
    # register address
    ACCEL_XOUT  = 0x3b
    ACCEL_YOUT  = 0x3d
    ACCEL_ZOUT  = 0x3f
    TEMP_OUT    = 0x41
    GYRO_XOUT   = 0x43
    GYRO_YOUT   = 0x45
    GYRO_ZOUT   = 0x47

    PWR_MGMT_1  = 0x6b
    PWR_MGMT_2  = 0x6c

    bus = smbus.SMBus(1)
    bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

    def __init__(self):
        pass

    #
    # Sub function
    #
    # 1byte read
    def read_byte(self,adr):
        return bus.read_byte_data(self.DEV_ADDR, adr)
    # 2byte read
    def read_word(self,adr):
        high = self.bus.read_byte_data(self.DEV_ADDR, adr)
        low = self.bus.read_byte_data(self.DEV_ADDR, adr+1)
        val = (high << 8) + low
        return val
    # Sensor data read
    def read_word_sensor(self,adr):
        val = self.read_word(adr)
        if (val >= 0x8000):         # minus
            return -((65535 - val) + 1)
        else:                       # plus
            return val

    def get_temp(self):
        temp = self.read_word_sensor(self.TEMP_OUT)
        x = temp / 340 + 36.53
        return x

    # get gyro data
    def get_gyro_data_lsb(self):
        x = self.read_word_sensor(self.GYRO_XOUT)
        y = self.read_word_sensor(self.GYRO_YOUT)
        z = self.read_word_sensor(self.GYRO_ZOUT)
        return [x, y, z]
    def get_gyro_data_deg(self):
        x,y,z = self.get_gyro_data_lsb()
        x = x / 131.0
        y = y / 131.0
        z = z / 131.0
        return [x, y, z]

    # get accel data
    def get_accel_data_lsb(self):
        x = self.read_word_sensor(self.ACCEL_XOUT)
        y = self.read_word_sensor(self.ACCEL_YOUT)
        z = self.read_word_sensor(self.ACCEL_ZOUT)
        return [x, y, z]
    # get accel data
    def get_accel_data_g(self):
        x,y,z = self.get_accel_data_lsb()
        x = x / 16384.0
        y = y / 16384.0
        z = z / 16384.0
        return [x, y, z]

mpu6050 = mpu6050()

def talker():
    acc_publisher  = rospy.Publisher('/mpu6050_acc',  Vector3, queue_size=10)
    gyro_publisher = rospy.Publisher('/mpu6050_gyro', Vector3, queue_size=10)
    temp_publisher = rospy.Publisher('/mpu6050_temp', Float32, queue_size=10)
    
    rospy.init_node('imu_sensor')
    r = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        acc_msg = Vector3()
        acc_array  = mpu6050.get_accel_data_g()
        acc_msg.x = acc_array[0]
        acc_msg.y = acc_array[1]
        acc_msg.z = acc_array[2]
        acc_publisher.publish(acc_msg)
        
        gyro_msg = Vector3()
        gyro_array = mpu6050.get_gyro_data_deg()
        gyro_msg.x = gyro_array[0]
        gyro_msg.y = gyro_array[1]
        gyro_msg.z = gyro_array[2]
        gyro_publisher.publish(gyro_msg)
        
        temp_msg = Float32()
        temp_msg = mpu6050.get_temp()
        temp_publisher.publish(temp_msg)
        
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass