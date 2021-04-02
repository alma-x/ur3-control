#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <ctime>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <gazebo_msgs/ModelState.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include "std_msgs/String.h"
#include <boost/thread/thread.hpp>

using namespace std;
using namespace geometry_msgs;
using namespace moveit;
using namespace planning_interface;
using namespace moveit_msgs;

double tol=0.01;
unsigned int attempt=10000;
static const string PLANNING_GROUP = "manipulator";
bool success;
int alpha,beta,positivita=1;
MoveGroupInterface *robot;
MoveGroupInterface::Plan my_plan;
vector<double> joint_group_positions;
vector<CollisionObject> collision_objects;
string CollisionObjectID;
vector<string> object_ids;
ros::Publisher planning_scene_diff_publisher;
ros:: Publisher gazebo_model_state_pub;
char joystick_input_char='0';
bool controller_bool=false;
bool picked=false;
string nome_oggetto;
char input_char;
bool joystick_ready;

void pick(string name_object);
char getch();
double grad_to_rad(double grad);
void tf2quat_to_pose(tf2::Quaternion q,Pose *p);
void SetPoseOrientationRPY(Pose *p,int x0,int y0,int z0);
void AddToPoseOrientationRPYInGradi(Pose *p,double roll,double pitch,double yaw);
void ruotagiunto(unsigned int giunto,int angolo);
void set_angolo(unsigned int giunto,int angolo);
void move_to_pose(geometry_msgs::Pose pt, bool Orientamento);
void stampa_Pose(Pose po);
void stampa_giunti();
void PosizioniBase(int posizione);
void publish_joystick_info();

void pick(string name_object){
  Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.2;
  pose.position.z = 0.0;
  pose.orientation.w = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;

  gazebo_msgs::ModelState model_state;
  model_state.model_name = name_object;
  model_state.pose = pose;
  model_state.reference_frame =std::string("wrist_3_link");

  picked=true;
  while(picked==true){
  gazebo_model_state_pub.publish(model_state);
  usleep(10000);
  }

}
char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}
double grad_to_rad(double grad)
{
  return grad*3.1415/180;
}
void tf2quat_to_pose(tf2::Quaternion q,Pose *p)
{
  p->orientation.x=q.getX();
  p->orientation.y=q.getY();
  p->orientation.z=q.getZ();
  p->orientation.w=q.getW();
}
void SetPoseOrientationRPY(Pose *p,int x0,int y0,int z0)
{
  tf2::Quaternion quat;
  quat.setRPY(grad_to_rad(x0),grad_to_rad(y0),grad_to_rad(z0));
  p->orientation.x=quat.getX();
  p->orientation.y=quat.getY();
  p->orientation.z=quat.getZ();
  p->orientation.w=quat.getW();
}
void AddToPoseOrientationRPYInGradi(Pose *p,double roll,double pitch,double yaw)
{
  tf2::Quaternion quat;
  tf::Quaternion q(
        p->orientation.x,
        p->orientation.y,
        p->orientation.z,
        p->orientation.w);
  tf::Matrix3x3 m(q);
  double r0, p0, y0;
  m.getRPY(r0, p0, y0);

  quat.setRPY(grad_to_rad(roll)+r0,grad_to_rad(pitch)+p0,grad_to_rad(yaw)+y0);
  p->orientation.x=quat.getX();
  p->orientation.y=quat.getY();
  p->orientation.z=quat.getZ();
  p->orientation.w=quat.getW();
}
void ruotagiunto(unsigned int giunto,int angolo){


  joint_group_positions=robot->getCurrentJointValues();
  joint_group_positions[giunto] = joint_group_positions[giunto]+grad_to_rad(angolo);  // radians
  robot->setJointValueTarget(joint_group_positions);


  success = (robot->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
  robot->move();

  }
void set_angolo(unsigned int giunto,int angolo)
{


  joint_group_positions=robot->getCurrentJointValues();
  joint_group_positions[giunto] = grad_to_rad(angolo);
  robot->setJointValueTarget(joint_group_positions);


  success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
  robot->move();
}
void move_to_pose(geometry_msgs::Pose pt, bool Orientamento){

  if(!Orientamento)
    robot->setPositionTarget(pt.position.x,pt.position.y,pt.position.z);
  else
    robot->setPoseTarget(pt);
  success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Provo a muovermi fino a dove mi hai detto tu, Risultato:%s", success ? "SUCCESS" : "FAILED");
  robot->execute(my_plan);
}
void stampa_Pose(Pose po)
{
  cout<<"Position"<<endl<<"X:"<<po.position.x<<endl<<"Y:"<<po.position.y<<endl<<"Z:"<<po.position.z<<endl;
  cout<<endl<<"Orientation"<<endl<<"X:"<<po.orientation.x<<endl<<"Y:"<<po.orientation.y<<endl<<"Z:"<<po.orientation.z<<endl<<"W:"<<po.orientation.w<<endl;
}
void stampa_giunti()
{
  joint_group_positions=robot->getCurrentJointValues();
  cout<<endl<<"Giunti:"<<endl;
  for(int i=0;i<joint_group_positions.size();i++){
    cout<<i<<":"<<joint_group_positions[i]<<endl;
  }
}
void PosizioniBase(int posizione){
  Pose target;
  if(posizione<=2){
    joint_group_positions=robot->getCurrentJointValues();
    for(unsigned int i=0;i<6;i++)
      joint_group_positions[i]=0;
    if(posizione==2){
      joint_group_positions[1]=grad_to_rad(-90);
      joint_group_positions[4]=grad_to_rad(90);
    }
    robot->setJointValueTarget(joint_group_positions);
  }
  //testa in basso
  if(posizione==3)
  {
    target.position=robot->getCurrentPose().pose.position;
    SetPoseOrientationRPY(&target,0,-270,0);
    robot->setPoseTarget(target);

  }
  //testa in alto
  if(posizione==4)
  {
    target.position=robot->getCurrentPose().pose.position;
    SetPoseOrientationRPY(&target,0,-90,0);
    robot->setPoseTarget(target);

  }
  success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
  //robot->move();
  robot->execute(my_plan);

}
void controlla(int key_int){
    string k;
    Pose targ=robot->getCurrentPose().pose;
    char key=key_int;
    unsigned int giunto=0;
    int angolo_base=3;
    bool bool_joint=false,bool_ee=false;
    double step=0.02,orientationstep=5;

    switch(key){
    case '0':{//positività
      positivita=positivita*(-1);
      break;
    }
    case 'q':{//x
      bool_ee=true;
      targ.position.x+=step;
      break;
    }
    case 'w':{//-x
      bool_ee=true;
      targ.position.x-=step;
      break;
    }
    case 'a':{//y
      bool_ee=true;
      targ.position.y+=step;
      break;
    }
    case 's':{//-y
      bool_ee=true;
      targ.position.y-=step;
      break;
    }
    case 'z':{//z
      bool_ee=true;
      targ.position.z+=step;
      break;
    }
    case 'x':{//-z
      bool_ee=true;
      targ.position.z-=step;
      break;
    }
    case 'p':{//-roll
      bool_ee=true;
      AddToPoseOrientationRPYInGradi(&targ,-orientationstep,0,0);
      break;
    }
    case 'o':{//roll
      bool_ee=true;
      AddToPoseOrientationRPYInGradi(&targ,orientationstep,0,0);
      break;
    }
    case 'l':{//-pitch
      bool_ee=true;
      AddToPoseOrientationRPYInGradi(&targ,0,-orientationstep,0);
      break;
    }
    case 'k':{//pitch
      bool_ee=true;
      AddToPoseOrientationRPYInGradi(&targ,0,orientationstep,0);
      break;
    }
    case 'm':{//-yaw
      bool_ee=true;
      AddToPoseOrientationRPYInGradi(&targ,0,0,-orientationstep);
      break;
    }
    case 'n':{//yaw
      bool_ee=true;
      AddToPoseOrientationRPYInGradi(&targ,0,0,orientationstep);
      break;
    }

    /*case 'h':{

    cout<<endl<<endl<<"Info:"<<endl;
    cout<<"q:ee moves along x"<<endl;
    cout<<"w:ee moves along -x"<<endl;
    cout<<"a:ee moves along y"<<endl;
    cout<<"s:ee moves along -y"<<endl;
    cout<<"z:ee moves along z"<<endl;
    cout<<"x:ee moves along -z"<<endl;
    cout<<"o:ee rotate along roll"<<endl;
    cout<<"p:ee rotate along -roll"<<endl;
    cout<<"k:ee rotate along pitch"<<endl;
    cout<<"l:ee rotate along -pitch"<<endl;
    cout<<"n:ee rotate along yaw"<<endl;
    cout<<"m:ee rotate along -yaw"<<endl;
    cout<<"e:EXIT"<<endl;
    break;
    }*/


    }

    if('1'<=key && key<='6'){
      giunto=key-'1';
      bool_joint=true;
    }

    if(bool_joint){
      ruotagiunto(giunto,positivita*angolo_base);
    }
    else{
      if(bool_ee){

        vector<Pose> waypoints;
        waypoints.push_back(targ);
        moveit_msgs::RobotTrajectory trajectory;
        robot->computeCartesianPath(waypoints,0.001,0,trajectory);
        success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
        my_plan.trajectory_=trajectory;
        robot->execute(my_plan);
        waypoints.clear();
      }
    }
}
void publish_joystick_info(){
  cout<<endl<<endl<<"Info:"<<endl;
  cout<<"q:ee moves along x"<<endl;
  cout<<"w:ee moves along -x"<<endl;
  cout<<"a:ee moves along y"<<endl;
  cout<<"s:ee moves along -y"<<endl;
  cout<<"z:ee moves along z"<<endl;
  cout<<"x:ee moves along -z"<<endl;
  cout<<"o:ee rotate along roll"<<endl;
  cout<<"p:ee rotate along -roll"<<endl;
  cout<<"k:ee rotate along pitch"<<endl;
  cout<<"l:ee rotate along -pitch"<<endl;
  cout<<"n:ee rotate along yaw"<<endl;
  cout<<"m:ee rotate along -yaw"<<endl;
  cout<<"e:EXIT"<<endl;

  ROS_INFO("\n\nPremi h, per info.Da qui in poi inserisci i comandi:");
}

