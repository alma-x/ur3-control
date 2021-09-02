#!/usr/bin/env python3
#import sys
#import copy
import rospy
#import moveit_msgs.msg
#import geometry_msgs.msg
#from math import pi
#from std_msgs.msg import String
#from moveit_commander.conversions import pose_to_list
from ur3_control.srv import collision_object_srv,collision_object_srvResponse, collision_object_srvRequest, float_return_srv, float_return_srvResponse
import sympy as sym

## END_SUB_TUTORIAL
bool_exit=False

def callback_service(req):
    global bool_exit
    #print('Collision service received')

    x=sym.Symbol('x')
    f=sym.Eq(x**2+2*x+1,0)
    solution=sym.solve(f,x)
    res1=solution[0]
    res2=0
    return float_return_srvResponse(result1=res1,result2=res2)


def main():
  rospy.init_node('collision_interface_node', anonymous=True)
  try:
    rospy.Service('solve_equation_serv', float_return_srv, callback_service)
    while (not rospy.core.is_shutdown()) and (not bool_exit):
        rospy.rostime.wallsleep(0.5)
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  #except bool_exit==True:
      #return
if __name__ == '__main__':
  main()
