#!/usr/bin/env python
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
from PyQt5 import uic, QtWidgets,QtGui
import rospy
from std_msgs.msg import String
from ur3_control.srv import UserInterface,UserInterfaceRequest,aruco_service,aruco_serviceRequest
from geometry_msgs.msg import Pose
from tf import transformations
import rospkg
import time

def menu():
    choice=-1
    while(choice!=0):
        print("select the action:")
        print("0) shut down")
        print("1) base position")
        print("2) change target aruco")
        print("3) reach target")
        print("4) gripper")
        print("4) joystick")

        choice=input("choice: ")

        if(choice==0):
            modality='exit'
            target_pose=Pose()
            target_joints=[0,0,0,0,0,0]
            second_information='null'
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)

        elif(choice==1):
            target_joints=[0,0,0,0,0,0]
            target_pose=Pose()
            modality='automazione_go_pos_iniziale'
            second_information=''
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)

        elif(choice==2):
            target_joints=[0,0,0,0,0,0]
            target_pose=Pose()
            modality='automazione_pannello_nextAruco'
            second_information=str(input("target aruco:"))
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)

        elif(choice==3):
            target_joints=[0,0,0,0,0,0]
            target_pose=Pose()
            modality='automazione_pannello_MoveToSelectedAruco'
            second_information='null'
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)

        elif(choice==4):
            print("choose gripper position: ")
            print("1) open")
            print("2) semi_open")
            print("3) semi_close")
            print("4) close")

            gripper_command=input("gripper position: ")

            if(gripper_command==1):
                second_information="open"
            elif(gripper_command==2):
                second_information="semi_open"
            elif(gripper_command==3):
                second_information="semi_close"
            elif(gripper_command==4):
                second_information="close"

            target_joints=[0,0,0,0,0,0]
            target_pose=Pose()
            modality='controlla_gripper'
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)

        elif(choice==5):
            modality='joystick'
            target_pose=Pose()
            target_joints=[0,0,0,0,0,0]
            second_information='null'
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
            print('Attenzione, modalita joystick attiva. Il controllo e sull altro terminale')
            print('Prima di lanciare un altro comando, uscire dalla modalita joystick dall altro terminale')


def init():
    print("initialization")
    global serv,bridge_serv
    rospy.init_node('UI_gara', anonymous=True)
    serv = rospy.ServiceProxy('/user_interface_serv', UserInterface)
    bridge_serv= rospy.ServiceProxy('/aruco_modality', aruco_service)


if __name__ == "__main__":
    init()
    menu()

