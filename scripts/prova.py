#!/usr/bin/env python3


import rospy
from ur3_control.srv import *
from ur3_control.msg import cv_to_bridge as bridge_msg
def add_two_ints_server():

    rospy.init_node('add_two_ints_server')
    r = rospy.Rate(10) #10hz
    pub = rospy.Publisher('aruco_bridge_opencv', bridge_msg, queue_size=10)
    
    msg=bridge_msg()
    msg.success=True
    pub.publish(msg)

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()


if __name__ == "__main__":
    print("provaa")
    add_two_ints_server()

