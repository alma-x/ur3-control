#!/usr/bin/env python3
#import sys
#import copy
import rospy
#import moveit_msgs.msg
#import geometry_msgs.msg
from math import sin,cos,atan,pi
#from std_msgs.msg import String
#from moveit_commander.conversions import pose_to_list
from ur3_control.srv import collision_object_srv,collision_object_srvResponse, collision_object_srvRequest, float_return_srv, float_return_srvResponse
#import sympy as sym

## END_SUB_TUTORIAL
bool_exit=False

def callback_service(req):
    global bool_exit

    L=0.0874
    phi=0.1745
    #theta=sym.Symbol('theta')
    Xa=req.num1
    Ya=req.num2

    #f=sym.Eq(Ya+L*sym.cos(theta)-(sym.tan(theta+phi)*(Xa-L*sym.sin(theta))),0)
    #solution = sym.solve(f,theta)
    res1=2*atan((5000*Xa*cos(pi/18) + 5000*Ya*sin(pi/18) + (25000000*Xa**2*cos(pi/18)**2 - 190969*cos(pi/18)**2 + 25000000*Ya**2*cos(pi/18)**2 + 25000000*Xa**2*sin(pi/18)**2 + 25000000*Ya**2*sin(pi/18)**2)**(1/2))/(437*cos(pi/18) - 5000*Ya*cos(pi/18) + 5000*Xa*sin(pi/18)))
    res2=2*atan((5000*Xa*cos(pi/18) + 5000*Ya*sin(pi/18) - (25000000*Xa**2*cos(pi/18)**2 - 190969*cos(pi/18)**2 + 25000000*Ya**2*cos(pi/18)**2 + 25000000*Xa**2*sin(pi/18)**2 + 25000000*Ya**2*sin(pi/18)**2)**(1/2))/(437*cos(pi/18) - 5000*Ya*cos(pi/18) + 5000*Xa*sin(pi/18)))
    print("solution1:")
    print(res1)
    print("solution2:")
    print(res2)
    return float_return_srvResponse(result1=res1,result2=res2)


    #x=sym.Symbol('x')
    #f=sym.Eq(x**2+2*x+1,0)
    #solution=sym.solve(f,x)
    #res1=solution[0]
    #res2=0
    #return float_return_srvResponse(result1=res1,result2=res2)


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
