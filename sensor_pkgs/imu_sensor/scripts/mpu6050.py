#!/usr/bin/env python

import rospy
import smbus            # use I2C
import math             # mathmatics
from time import sleep  # time module
from sensor_msgs.msg import Imu, Temperature

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

def imu_publish():
    imu_publisher = rospy.Publisher('/mpu6050_imu', Imu, queue_size=10)
    temp_publisher = rospy.Publisher('/mpu6050_temp', Temperature, queue_size=10)
    
    rospy.init_node('imu_sensor')
    r = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        imu_msg = Imu()
        
        acc_array  = mpu6050.get_accel_data_g()
        imu_msg.linear_acceleration.x = acc_array[0]
        imu_msg.linear_acceleration.y = acc_array[1]
        imu_msg.linear_acceleration.z = acc_array[2]
        
        gyro_array = mpu6050.get_gyro_data_deg()
        imu_msg.angular_velocity.x = gyro_array[0]
        imu_msg.angular_velocity.y = gyro_array[1]
        imu_msg.angular_velocity.z = gyro_array[2]
        
        imu_publisher.publish(imu_msg)
        
        temp_msg = Temperature()
        temp_val = mpu6050.get_temp()
        temp_msg.temperature = temp_val
        temp_publisher.publish(temp_msg)
        
        r.sleep()

if __name__ == '__main__':
    try:
        imu_publish()
    except rospy.ROSInterruptException: pass