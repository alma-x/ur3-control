#!/usr/bin/env python3

from ur3_control.srv import aruco_service, aruco_serviceResponse
import rospy
x=False
def callback(req):
    if x :
        return aruco_serviceResponse(
                success=False,
                x=30.012,
                y=0.005,
                z=213
                )
    else :
        return aruco_serviceResponse(
                success=False,
                x=0,
                y=0,
                z=0
                )

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('aruco_service', aruco_service, callback)
    rospy.spin()

if __name__ == "__main__":
    print("Ready to add two ints.")
    add_two_ints_server()
