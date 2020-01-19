#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32

dist = 0.5
total_dist = 0
section_dist = 0

def mileage_publisher():
    global total_dist
    global section_dist
    total_dist_pub = rospy.Publisher('total_distance', Float32, queue_size=10)
    section_dist_pub = rospy.Publisher('section_distance', Float32, queue_size=10)
    rospy.init_node('mileage_publisher', anonymous=True)
    r = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        total_dist = total_dist + dist
        section_dist = section_dist + dist
        rospy.loginfo("total mileage   : %.1f", total_dist)
        rospy.loginfo("section mileage : %.1f", section_dist)
        total_dist_pub.publish(total_dist)
        section_dist_pub.publish(section_dist)
        r.sleep()

if __name__ == '__main__':
    try:
        mileage_publisher()
    except rospy.ROSInterruptException: 
        pass