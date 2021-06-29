#include <ros/ros.h>
#include <ros/package.h>
#include "ros/service.h"
#include "ur3_control/aruco_service.h"
#include "ur3_control/UserInterface.h"
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
#include <gazebo_msgs/GetModelState.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include "std_msgs/String.h"
#include <boost/thread/thread.hpp>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "tf_conversions/tf_eigen.h"
#include <fstream>

using namespace std;
using namespace geometry_msgs;
using namespace moveit;
using namespace planning_interface;
using namespace moveit_msgs;
using namespace Eigen;

double tol=0.01;
bool SDR_SOLIDALE=false;
unsigned int attempt=10000;
static const string PLANNING_GROUP = "manipulator";
bool success;
int alpha,beta,positivita=1;
MoveGroupInterface *robot;
MoveGroupInterface::Plan my_plan;
vector<CollisionObject> collision_objects;
string CollisionObjectID;
vector<string> object_ids;
ros::Publisher planning_scene_diff_publisher;
ros:: Publisher gazebo_model_state_pub;
ros::ServiceClient pose_object_client;
char joystick_input_char='0';
bool controller_bool=false;
bool picked=false;
string nome_oggetto;
char input_char;
bool joystick_ready;

int std_planning_time=4;

string str_md_stop_bpa="md_stop_blocca_al_primo_aruco";
string str_md_bpa="md_blocca_al_primo_aruco";
string str_md_rd="md_richiesta_dati";
string str_md_next_aruco="md_next_aruco";

string str_up="posizione_up";
string str_home="posizione_home";
string str_tb="posizione_testa_bassa";
string str_ta="posizione_testa_alta";
string str_r="posizione_adatta_rotazione";
string str_near_centrifuga="posizione_near_centrifuga";
string str_centrifuga="posizione_centrifuga";
string str_pannello="posizione_pannello";

vector<double> pos_joint_home;
vector<double> pos_joint_up;
vector<double> pos_joint_r;
vector<double> pos_joint_centrifuga;
vector<double> pos_joint_pannello;
vector<int> pos_rpy_tb;
vector<int> pos_rpy_ta;
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
bool PosizioniBase(string str_posizione);
void publish_joystick_info();
void take_object();
Pose pose_traslation_FAKE_solidale(Vector3d translation);
Pose pose_traslation_solidale(Vector3d translation);
void ruota_e_cerca_aruco();
void ruota_360();
void automatizzato();
double rad_to_grad(double rad);
bool callback_service_aruco_found(ur3_control::aruco_service::Request &req,ur3_control::aruco_service::Response &res);
bool individua_aruco(Pose *aruco_pose);
bool move_to_aruco();
ur3_control::aruco_serviceResponse bridge_service(string modalita,string second_information);
bool function_pose_aruco(ur3_control::aruco_serviceResponse msg_from_bridge);

void pick(string name_object){
  Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.11;
  pose.position.z = 0.0;
  pose.orientation.w = 1;
  pose.orientation.x = 1;
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
double rad_to_grad(double rad)
{
  return rad*180/3.1415;
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


  vector<double> joint_group_positions=robot->getCurrentJointValues();
  joint_group_positions[giunto] = joint_group_positions[giunto]+grad_to_rad(angolo);  // radians
  robot->setJointValueTarget(joint_group_positions);


  success = (robot->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
  robot->move();

  }
void set_angolo(unsigned int giunto,int angolo)
{


  vector<double> joint_group_positions=robot->getCurrentJointValues();
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
  ROS_INFO_NAMED("tutorial", "Risultato:%s", success ? "SUCCESS" : "FAILED");
  robot->execute(my_plan);
}
void stampa_Pose(Pose po)
{
  cout<<"Position"<<endl<<"X:"<<po.position.x<<endl<<"Y:"<<po.position.y<<endl<<"Z:"<<po.position.z<<endl;
  cout<<endl<<"Orientation"<<endl<<"X:"<<po.orientation.x<<endl<<"Y:"<<po.orientation.y<<endl<<"Z:"<<po.orientation.z<<endl<<"W:"<<po.orientation.w<<endl;
  tf::Quaternion q(
        po.orientation.x,
        po.orientation.y,
        po.orientation.z,
        po.orientation.w);
  tf::Matrix3x3 m(q);
  double r0, p0, y0;
  m.getRPY(r0, p0, y0);
  cout<<"r0:"<<rad_to_grad(r0)<<" p0:"<<rad_to_grad(p0)<<" y0:"<<rad_to_grad(y0)<<endl<<endl;
}
void stampa_giunti()
{
  vector<double> joint_group_positions;
  joint_group_positions=robot->getCurrentJointValues();
  cout<<endl<<"Giunti:"<<endl;
  for(int i=0;i<joint_group_positions.size();i++){
    cout<<i<<":"<<joint_group_positions[i]<<" xxx "<<rad_to_grad(joint_group_positions[i])<<endl;
  }
}
void PosizioniBase_old(int posizione){
  vector<double> joint_group_positions;
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
  robot->move();

}
bool PosizioniBase(string str_posizione){
  vector<double> joint_group_positions=robot->getCurrentJointValues();
  Pose target=robot->getCurrentPose().pose;
  if(str_posizione==str_home)
  {
    robot->setJointValueTarget(pos_joint_home);
  }
  else
  if(str_posizione==str_up)
  {
    robot->setJointValueTarget(pos_joint_up);
  }
  else
  if(str_posizione==str_ta)
  {

    SetPoseOrientationRPY(&target,pos_rpy_ta[0],pos_rpy_ta[1],pos_rpy_ta[2]);
    robot->setPoseTarget(target);
  }
  else
  if(str_posizione==str_tb)
  {

    SetPoseOrientationRPY(&target,pos_rpy_tb[0],pos_rpy_tb[1],pos_rpy_tb[2]);
    robot->setPoseTarget(target);
  }
  else
  if(str_posizione==str_r)
  {

    joint_group_positions[1] = pos_joint_r[1];
    joint_group_positions[2] = pos_joint_r[2];
    joint_group_positions[3] = pos_joint_r[3];
    joint_group_positions[4] = pos_joint_r[4];

    robot->setJointValueTarget(joint_group_positions);
  }
  else
  if(str_posizione==str_near_centrifuga){
    joint_group_positions[0] = pos_joint_centrifuga[0];  // radians
    robot->setJointValueTarget(joint_group_positions);
  }
  else
  if(str_posizione==str_centrifuga){
    robot->setJointValueTarget(pos_joint_centrifuga);
  }
  else
  if(str_posizione==str_pannello){
    robot->setJointValueTarget(pos_joint_pannello);
  }
  else{
    ROS_INFO("Posizione non presente tra quelle base");
    return false;
  }

  success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
  robot->move();
  return true;
}
void controlla(int key_int){
    Vector3d translation(0,0,0);
    string k;
    Pose targ,pose_robot=robot->getCurrentPose().pose;
    targ=pose_robot;
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
      Vector3d temp(step,0,0);
      translation=temp;
      break;
    }
    case 'w':{//-x
      bool_ee=true;
      targ.position.x-=step;
      Vector3d temp(-step,0,0);
      translation=temp;
      break;
    }
    case 'a':{//y
      bool_ee=true;
      targ.position.y+=step;
      Vector3d temp(0,step,0);
      translation=temp;
      break;
    }
    case 's':{//-y
      bool_ee=true;
      targ.position.y-=step;
      Vector3d temp(0,-step,0);
      translation=temp;
      break;
    }
    case 'z':{//z
      bool_ee=true;
      targ.position.z+=step;
      Vector3d temp(0,0,step);
      translation=temp;
      break;
    }
    case 'x':{//-z
      bool_ee=true;
      targ.position.z-=step;
      Vector3d temp(0,0,-step);
      translation=temp;
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
    case 'r':{
      SDR_SOLIDALE=!SDR_SOLIDALE;
      ROS_INFO("Cambio sistema di riferimento\n Ora:%s\n",(SDR_SOLIDALE)?"SRD_SOLIDALE":"SDR non solidale");
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
        if(!SDR_SOLIDALE){
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
        else{
          tf2::Quaternion quat1;
          tf::Quaternion q1(
                pose_robot.orientation.x,
                pose_robot.orientation.y,
                pose_robot.orientation.z,
                pose_robot.orientation.w);
          tf::Matrix3x3 m1(q1);
          double r0_first, p0_first, y0_first;
          m1.getRPY(r0_first,p0_first,y0_first);
          quat1.setRPY(0,p0_first,y0_first);



          pose_robot.orientation.x=quat1.getX();
          pose_robot.orientation.y=quat1.getY();
          pose_robot.orientation.z=quat1.getZ();
          pose_robot.orientation.w=quat1.getW();



          Affine3d T_actual;
          tf::Pose pose_robot_tf;
          tf::poseMsgToTF(pose_robot,pose_robot_tf);
          tf::poseTFToEigen(pose_robot_tf,T_actual);
          T_actual.translate(translation);
          tf::poseEigenToTF(T_actual,pose_robot_tf);
          tf::poseTFToMsg(pose_robot_tf,pose_robot);




          tf2::Quaternion quat2;
          tf::Quaternion q2(
                pose_robot.orientation.x,
                pose_robot.orientation.y,
                pose_robot.orientation.z,
                pose_robot.orientation.w);
          tf::Matrix3x3 m2(q2);
          double r0_second, p0_second, y0_second;
          m2.getRPY(r0_second,p0_second,y0_second);
          quat2.setRPY(r0_first,p0_second,y0_second);//r0_first per lasciarlo al vecchio roll




          pose_robot.orientation.x=quat2.getX();
          pose_robot.orientation.y=quat2.getY();
          pose_robot.orientation.z=quat2.getZ();
          pose_robot.orientation.w=quat2.getW();



          move_to_pose(pose_robot,true);
        }
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
void take_object(){
  tf2::Quaternion quat;
  Pose target;
  gazebo_msgs::GetModelState getmodelstate;
  cout<<endl<<"nome oggetto: ";
  cin>>nome_oggetto;
  getmodelstate.request.model_name=nome_oggetto;
  pose_object_client.call(getmodelstate);
  cout<<endl<<"posizione di "<<nome_oggetto<<endl;
  cout<<"X: "<<getmodelstate.response.pose.position.x<<endl;
  cout<<"Y: "<<getmodelstate.response.pose.position.y<<endl;
  cout<<"Z: "<<getmodelstate.response.pose.position.z<<endl;
  cout<<endl<<"orientazione: "<<endl;
  cout<<"X: "<<getmodelstate.response.pose.orientation.x<<endl;
  cout<<"Y: "<<getmodelstate.response.pose.orientation.y<<endl;
  cout<<"Z: "<<getmodelstate.response.pose.orientation.z<<endl;
  cout<<"W: "<<getmodelstate.response.pose.orientation.w<<endl;

 //PosizioniBase(2);
 //usleep(5000000);
  target.position.x=getmodelstate.response.pose.position.x;
  target.position.y=getmodelstate.response.pose.position.y;
  target.position.z=getmodelstate.response.pose.position.z+0.05;
  quat.setRPY(grad_to_rad(0),grad_to_rad(90),grad_to_rad(0));
  tf2quat_to_pose(quat,&target);
  move_to_pose(target,true);
}
Pose pose_traslation_FAKE_solidale(Vector3d translation)
{
  Pose pose_robot=robot->getCurrentPose().pose;
  tf2::Quaternion quat1;
  tf::Quaternion q1(
        pose_robot.orientation.x,
        pose_robot.orientation.y,
        pose_robot.orientation.z,
        pose_robot.orientation.w);
  tf::Matrix3x3 m1(q1);
  double r0_first, p0_first, y0_first;
  m1.getRPY(r0_first,p0_first,y0_first);
  quat1.setRPY(0,p0_first,y0_first);

  pose_robot.orientation.x=quat1.getX();
  pose_robot.orientation.y=quat1.getY();
  pose_robot.orientation.z=quat1.getZ();
  pose_robot.orientation.w=quat1.getW();

  Affine3d T_actual;
  tf::Pose pose_robot_tf;
  tf::poseMsgToTF(pose_robot,pose_robot_tf);
  tf::poseTFToEigen(pose_robot_tf,T_actual);
  T_actual.translate(translation);
  tf::poseEigenToTF(T_actual,pose_robot_tf);
  tf::poseTFToMsg(pose_robot_tf,pose_robot);

  tf2::Quaternion quat2;
  tf::Quaternion q2(
        pose_robot.orientation.x,
        pose_robot.orientation.y,
        pose_robot.orientation.z,
        pose_robot.orientation.w);
  tf::Matrix3x3 m2(q2);
  double r0_second, p0_second, y0_second;
  m2.getRPY(r0_second,p0_second,y0_second);
  quat2.setRPY(r0_first,p0_second,y0_second);//r0_first per lasciarlo al vecchio roll

  pose_robot.orientation.x=quat2.getX();
  pose_robot.orientation.y=quat2.getY();
  pose_robot.orientation.z=quat2.getZ();
  pose_robot.orientation.w=quat2.getW();

  return pose_robot;
}
Pose pose_traslation_solidale(Vector3d translation)
{
  Pose pose_robot=robot->getCurrentPose().pose;


  Affine3d T_actual;
  tf::Pose pose_robot_tf;
  tf::poseMsgToTF(pose_robot,pose_robot_tf);
  tf::poseTFToEigen(pose_robot_tf,T_actual);
  T_actual.translate(translation);
  tf::poseEigenToTF(T_actual,pose_robot_tf);
  tf::poseTFToMsg(pose_robot_tf,pose_robot);

  return pose_robot;
}
void ruota_360_old(){
  ROS_INFO("Starting rotating 360");




  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
//const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  ROS_INFO("kinematic model aquired");

  //rotazione

  vector<double> joint_positions;
  joint_positions=robot->getCurrentJointValues();



  double angle_resolution=1;
  vector<Pose> waypoints;
  for (int i= 0; i< 360/angle_resolution; i++)
    {
      joint_positions[0] = joint_positions[0]+grad_to_rad(angle_resolution);  // radians


      kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);


      tf::Pose target_tf;
      Pose target;
      const Eigen::Affine3d& link_pose = kinematic_state->getGlobalLinkTransform("tool0");//forse tool0
      tf::poseEigenToTF(link_pose,target_tf);
      tf::poseTFToMsg(target_tf,target);
      SetPoseOrientationRPY(&target,0,90,0);
      waypoints.push_back(target);
    }

      moveit_msgs::RobotTrajectory trajectory;
      robot->computeCartesianPath(waypoints,0.0001,0,trajectory);

      success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
      my_plan.trajectory_=trajectory;
      robot->execute(my_plan);

      ROS_INFO("Rotazione 360 completata");

}
void ruota_360(){
  ROS_INFO("Starting rotating 360");



  vector<double> joint_positions;
  joint_positions=robot->getCurrentJointValues();
  joint_positions[0] = joint_positions[0]+grad_to_rad(360);  // radians
  robot->setJointValueTarget(joint_positions);
  success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);

  if(!success){

    joint_positions=robot->getCurrentJointValues();
    joint_positions[0] = joint_positions[0]+grad_to_rad(270);  // radians
    robot->setJointValueTarget(joint_positions);
    success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);

    if(!success){

      joint_positions=robot->getCurrentJointValues();
      joint_positions[0] = joint_positions[0]+grad_to_rad(180);  // radians
      robot->setJointValueTarget(joint_positions);
      success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);

      if(!success){

        joint_positions=robot->getCurrentJointValues();
        joint_positions[0] = joint_positions[0]+grad_to_rad(90);  // radians
        robot->setJointValueTarget(joint_positions);
        success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
      }
    }

  }
  ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
  robot->execute(my_plan);

  ROS_INFO("Rotazione 360 completata");

}
void ruota_e_cerca_aruco(){
    /*
  robot->setPlanningTime(0.2);
  ros::NodeHandle node_handle;
  ros::ServiceClient client1;
  client1 = node_handle.serviceClient<ur3_control::aruco_service>("/aruco_service");
  //bool aruco_trovato=false;
  int angolo=10;
  int cont=0;

  ur3_control::aruco_service aruco_srv;
  do{
  cont++;
  joint_group_positions=robot->getCurrentJointValues();
  joint_group_positions[1] = grad_to_rad(-120);
  joint_group_positions[2] = grad_to_rad(100);
  joint_group_positions[3] = grad_to_rad(-100);
  joint_group_positions[4] = grad_to_rad(-90);
  if(cont!=0){
    joint_group_positions[0] = joint_group_positions[0]+grad_to_rad(angolo);  // radians
  }
  robot->setJointValueTarget(joint_group_positions);
  success = (robot->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  robot->move();

  client1.call(aruco_srv);
  ROS_INFO("Aruco found:%s",(aruco_srv.response.success ? "YES":"NO"));

  }while((!aruco_srv.response.success) && (cont!=(360/angolo)));

  robot->setPlanningTime(3);//resetto il planning time abbassato per eseguire i precedenti movimenti

  ROS_INFO("Aruco position: x=%f y=%f z=%f",aruco_srv.response.x,aruco_srv.response.y,aruco_srv.response.z);
  Pose target_aruco;
  Vector3d camera_block_vector(aruco_srv.response.x,aruco_srv.response.y,aruco_srv.response.z);
  target_aruco=pose_traslation_solidale(camera_block_vector);

  //target_aruco.position.z=target_aruco.position.z+0.01;//evita collisione
  char scelta;
  scelta='y';
  if(scelta=='y'){
    SetPoseOrientationRPY(&target_aruco,0,-270,0);
  }
  else{
    //from linear vector to Matrix
    Matrix3d R_cam_aruco;
    {
    R_cam_aruco(0,0)=aruco_srv.response.vector[0]; R_cam_aruco(0,1)=aruco_srv.response.vector[1]; R_cam_aruco(0,2)=aruco_srv.response.vector[2];
    R_cam_aruco(1,0)=aruco_srv.response.vector[3]; R_cam_aruco(1,1)=aruco_srv.response.vector[4]; R_cam_aruco(1,2)=aruco_srv.response.vector[5];
    R_cam_aruco(2,0)=aruco_srv.response.vector[6]; R_cam_aruco(2,1)=aruco_srv.response.vector[7]; R_cam_aruco(2,2)=aruco_srv.response.vector[8];
    }
    for(int r=0;r<3;r++){
      for(int c=0;c<3;c++){
        printf("%f ",R_cam_aruco(r,c));
      }
      printf("\n");
    }

    Matrix3d R_w3_cam,R_w3_cam_rot_y,R_w3_cam_rot_x;
    {
    R_w3_cam_rot_y(0,0)=0; R_w3_cam_rot_y(0,1)=0; R_w3_cam_rot_y(0,2)=-1;
    R_w3_cam_rot_y(1,0)=0; R_w3_cam_rot_y(1,1)=1; R_w3_cam_rot_y(1,2)=0;
    R_w3_cam_rot_y(2,0)=1; R_w3_cam_rot_y(2,1)=0; R_w3_cam_rot_y(2,2)=0;

    R_w3_cam_rot_x(0,0)=1; R_w3_cam_rot_x(0,1)=0; R_w3_cam_rot_x(0,2)=0;
    R_w3_cam_rot_x(1,0)=0; R_w3_cam_rot_x(1,1)=0; R_w3_cam_rot_x(1,2)=1;
    R_w3_cam_rot_x(2,0)=0; R_w3_cam_rot_x(2,1)=-1; R_w3_cam_rot_x(2,2)=0;

    R_w3_cam=R_w3_cam_rot_y*R_w3_cam_rot_x;
    }


    Pose pose_robot=robot->getCurrentPose().pose;
    Affine3d T_actual;
    tf::Pose pose_robot_tf;
    tf::poseMsgToTF(pose_robot,pose_robot_tf);
    tf::poseTFToEigen(pose_robot_tf,T_actual);

    Matrix3d R_0_w3,R_0_cam,R_0_aruco;
    R_0_w3=T_actual.rotation();
    R_0_cam=R_0_w3*R_w3_cam;
    R_0_aruco=R_0_cam*R_cam_aruco;


    tf::Matrix3x3 M;
    /// Converts an Eigen Quaternion into a tf Matrix3x3
    matrixEigenToTF(R_0_aruco, M);

    double r0, p0, y0;
    M.getRPY(r0, p0, y0);
    tf2::Quaternion quat;
    quat.setRPY(r0,p0,y0);

    target_aruco.orientation.x=quat.getX();
    target_aruco.orientation.y=quat.getY();
    target_aruco.orientation.z=quat.getZ();
    target_aruco.orientation.w=quat.getW();


  }

  move_to_pose(target_aruco,true);

*/
}

void trasformazioni(){
  Pose pose_robot=robot->getCurrentPose().pose;

  ROS_INFO("Traslazione:");
  ROS_INFO("x: %f\n y:%f\n z:%f\n ",pose_robot.position.x,pose_robot.position.y,pose_robot.position.z);

  ROS_INFO("Rotazione:");
  ROS_INFO("x: %f\n y:%f\n z:%f\n w:%f\n",pose_robot.orientation.x,pose_robot.orientation.y,pose_robot.orientation.z,pose_robot.orientation.w);

  Pose pose_robot_target;
  Affine3d T_0_tool,T_tool_camera,T_0_camera;
  tf::Pose pose_robot_tf,pose_target_tf;
  tf::poseMsgToTF(pose_robot,pose_robot_tf);
  tf::poseTFToEigen(pose_robot_tf,T_0_tool);

  cout<<"T_0_tool translation:"<<endl<<T_0_tool.translation()<<endl;
  cout<<"T_0_tool rotation:"<<endl<<T_0_tool.rotation()<<endl;

  /*
   *
   *    tra ee_link e camera_realsense_gazebo
        A=np.array([[0,1,0,0.4569],[1,0,0,0.19425],[0,0,-1,0.6655],[0,0,0,1]])
        B=np.array([[0,0,1,0.025],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
        p=np.array([[0],[0],[0],[1]])
        T=A.dot(B)
        result=T.dot(p)
*/
  Vector3d translation_tool_camera(0.025,0,0);
  Matrix3d rotation_tool_camera;
  Vector3d xvec_des(0,1,0),yvec_des(0,0,1),zvec_des(1,0,0);
  rotation_tool_camera.col(0)=xvec_des;
  rotation_tool_camera.col(1)=yvec_des;
  rotation_tool_camera.col(2)=zvec_des;


  T_tool_camera.translation()=translation_tool_camera;
  T_tool_camera.linear()=rotation_tool_camera;
  cout<<"T_tool_camera translation:"<<endl<<T_tool_camera.translation()<<endl;
  cout<<"T_tool_camera rotation:"<<endl<<T_tool_camera.rotation()<<endl;
  //ROS_INFO("T_tool_camera: %f %f %f",T_tool_camera.translation().x(),T_tool_camera.translation().y(),T_tool_camera.translation().z());


  T_0_camera=T_0_tool*T_tool_camera;

  tf::poseEigenToTF(T_0_camera,pose_target_tf);
  tf::poseTFToMsg(pose_target_tf,pose_robot_target);

  ROS_INFO("Traslazione:");
  ROS_INFO("x: %f\n y:%f\n z:%f\n ",pose_robot_target.position.x,pose_robot_target.position.y,pose_robot_target.position.z);

  ROS_INFO("Rotazione:");
  ROS_INFO("x: %f\n y:%f\n z:%f\n w:%f\n",pose_robot_target.orientation.x,pose_robot_target.orientation.y,pose_robot_target.orientation.z,pose_robot_target.orientation.w);

}
void reshape(){
    /*
  Matrix3d new_vector;
  {
  int rows,columns,dimensione,i=0,c=0,r=-1;
  rows=aruco_srv.response.dim[0];
  columns=aruco_srv.response.dim[1];
  dimensione=rows*columns;
  do{
    if(c%columns==0){
      c=0;
      r++;
    }
    new_vector=aruco_srv.response.vector[i];
    c++;
    i++;
  }while(i!=dimensione);
  for(r=0;r<rows;r++){
    for(c=0;c<rows;c++){
      printf("%f ",new_vector[r][c]);
    }
    printf("\n");
  }
  }*/
}
void automatizzato(){


  bridge_service(str_md_bpa,"");

  PosizioniBase(str_r);//si mette in posizione prima di iniziare a ruotare

  if(!move_to_aruco())//controlla se nell'attuale posizione c'è un aruco e nel caso lo piglia
  {
    ruota_360();
    if(!move_to_aruco())//controlla se nell'attuale posizione c'è un aruco e nel caso lo piglia
    {
      ROS_INFO("Aruco non trovato :(");
    }
  }

  bridge_service(str_md_stop_bpa,"");
}
bool individua_aruco(Pose *aruco_pose_solidale){
//
  ur3_control::aruco_serviceResponse aruco_srv_msg_resp=bridge_service(str_md_rd,"");
  ROS_INFO("Aruco found:%s",(aruco_srv_msg_resp.aruco_found ? "YES":"NO"));
  
  if(aruco_srv_msg_resp.aruco_found){
    ROS_INFO("ATTENZIONE E' ROTTA");
    return true;
  }
  else{
    return false;
  }


}
bool move_to_aruco(){
  Pose aruco_pose_solidale;
  if(individua_aruco(&aruco_pose_solidale)){

    robot->setPlanningTime(10);
    move_to_pose(aruco_pose_solidale,true);
    robot->setPlanningTime(std_planning_time);
    return true;
  }
  else {
    return false;
  }
}
void move_near_to_centrifuga(){


 PosizioniBase(str_r);
 PosizioniBase(str_near_centrifuga);



}
void move_to_centrifuga(){

  move_near_to_centrifuga();
  PosizioniBase(str_centrifuga);

}
bool move_to_pannello_aruco(){

  Pose aruco_pose_solidale;
  if(individua_aruco(&aruco_pose_solidale)){

    robot->setPlanningTime(10);
    move_to_pose(aruco_pose_solidale,true);
    robot->setPlanningTime(std_planning_time);
    return true;
  }
  else {
    return false;
  }
}
void aruco_pannello(){
  bool continua_=true;
  Pose temp;

  while(continua_){

    PosizioniBase(str_pannello);
    
    function_pose_aruco(bridge_service(str_md_rd,""));

    if(bridge_service(str_md_next_aruco,"").moreTargets<=0){
        continua_=false;
    }

  }
//muoviti sul primo aruco
//ritorna in un buon punto
//cambia aruco. se non c'è next esci

}
ur3_control::aruco_serviceResponse bridge_service(string modalita,string second_information){

    ros::NodeHandle node_handle;
    ros::ServiceClient client1;
    client1 = node_handle.serviceClient<ur3_control::aruco_service>("/aruco_modality");
    ur3_control::aruco_service aruco_srv_msg;

    aruco_srv_msg.request.modality=modalita;
    aruco_srv_msg.request.second_information=second_information;
    client1.call(aruco_srv_msg);
    return aruco_srv_msg.response;
}
void load_parameters()
{


    ifstream inFile;
    string nome,tipo;
    string pkgpath = ros::package::getPath("ur3_control");
    string path_txt="/txt/standard_positions.txt";
    string path_total=pkgpath + path_txt;
    cout<<"percorso txt: "<<path_total<<endl<<endl;
    inFile.open(path_total);
    if (!inFile) {
        cerr << "Unable to open file datafile.txt";
        exit(1);   // call system to stop
    }

    pos_joint_home=robot->getCurrentJointValues();
    pos_joint_up=robot->getCurrentJointValues();
    pos_joint_r=robot->getCurrentJointValues();
    pos_joint_centrifuga=robot->getCurrentJointValues();
    pos_joint_pannello=robot->getCurrentJointValues();
    if(pos_rpy_ta.size()==0){

        pos_rpy_ta.push_back(0);
        pos_rpy_ta.push_back(0);
        pos_rpy_ta.push_back(0);

    }
    if(pos_rpy_tb.size()==0){
        pos_rpy_tb.push_back(0);
        pos_rpy_tb.push_back(0);
        pos_rpy_tb.push_back(0);
    }
    while (inFile >> nome >> tipo) {
        if(tipo=="joint"){
            int j[6];
            for(int i=0;i<6;i++){
                inFile>>j[i];
                printf("%d:%d ",i,j[i]);
            }
            printf("\n");

            if(nome==str_home){
                for (int i=0;i<6;i++) {
                    pos_joint_home[i]=grad_to_rad(j[i]);
                }
            }
            if(nome==str_up){
                for (int i=0;i<6;i++) {
                    pos_joint_up[i]=grad_to_rad(j[i]);
                }
            }
            if(nome==str_r){
                for (int i=0;i<6;i++) {
                    pos_joint_r[i]=grad_to_rad(j[i]);
                }
            }
            if(nome==str_centrifuga){
                for (int i=0;i<6;i++) {
                    pos_joint_centrifuga[i]=grad_to_rad(j[i]);
                }
            }
            if(nome==str_pannello){
                for (int i=0;i<6;i++) {
                    pos_joint_pannello[i]=grad_to_rad(j[i]);
                }
            }

        }
        if(tipo=="rpy"){
            int r0,p0,y0;
            inFile>>r0>>p0>>y0;
            if(nome==str_ta){
                pos_rpy_ta[0]=r0;
                pos_rpy_ta[1]=p0;
                pos_rpy_ta[2]=y0;
                printf("rpy: %d %d %d\n",pos_rpy_ta[0],pos_rpy_ta[1],pos_rpy_ta[2]);
            }
            if(nome==str_tb){

                pos_rpy_tb[0]=r0;
                pos_rpy_tb[1]=p0;
                pos_rpy_tb[2]=y0;

                printf("rpy: %d %d %d\n",pos_rpy_tb[0],pos_rpy_tb[1],pos_rpy_tb[2]);
                }

        }
    }

    inFile.close();
}
bool function_pose_aruco(ur3_control::aruco_serviceResponse msg_from_bridge){

  robot->setPlanningTime(10);
  if(msg_from_bridge.aruco_found){

    Pose pose_robot=robot->getCurrentPose().pose;

    Pose pose_robot_target,pose_finalpos,pose_aruco,pose_camera;
    Affine3d T_0_tool,T_tool_camera,T_0_camera,T_camera_aruco,T_0_aruco,T_aruco_finalpos,T_0_finalpos,T_0_camera_gazebo,T_camera_camera_gazebo;
    tf::Pose pose_robot_tf,pose_target_tf,pose_finalpos_tf,pose_aruco_tf,pose_camera_tf;
    tf::poseMsgToTF(pose_robot,pose_robot_tf);
    tf::poseTFToEigen(pose_robot_tf,T_0_tool);
    //In questo punto ho T_0_tool

/*
 * old
    Vector3d translation_tool_camera(0.025,0,0);
    Matrix3d rotation_tool_camera;
    Vector3d xvec_des(0,1,0),yvec_des(0,0,1),zvec_des(1,0,0);
    rotation_tool_camera.col(0)=xvec_des;
    rotation_tool_camera.col(1)=yvec_des;
    rotation_tool_camera.col(2)=zvec_des;

    T_tool_camera.translation()=translation_tool_camera;
    T_tool_camera.linear()=rotation_tool_camera;

    T_0_camera=T_0_tool*T_tool_camera;
    //In questo punto ho T_0_camera
*/
/*
    0.9848         0    0.1736    0.0081
   -0.1736         0    0.9848    0.0874
         0   -1.0000         0         0
         0         0         0    1.0000
*/
    //from tool0 to camera mount        <origin xyz="0.0 0.0 -0.0015" rpy="1.5707 0.0 -1.5707" />
    //from camera mount to camera_link1 <origin xyz="0 -0.000447 0.08739618542" rpy="0 0 ${pi/2}" />
    //from camera link1 to camera link  <origin xyz="0.0 0.0 0.0" rpy="0 ${pi/18} 0" />
    Vector3d translation_tool_camera(0.0081,0.0874,0);
    Matrix3d rotation_tool_camera;
    Vector3d xvec_des(0.9848,-0.1736,0),yvec_des(0,0,-1),zvec_des(0.1736,0.9848,0);
    rotation_tool_camera.col(0)=xvec_des;
    rotation_tool_camera.col(1)=yvec_des;
    rotation_tool_camera.col(2)=zvec_des;

    T_tool_camera.translation()=translation_tool_camera;
    T_tool_camera.linear()=rotation_tool_camera;

    T_0_camera=T_0_tool*T_tool_camera;
    tf::poseEigenToTF(T_0_camera,pose_camera_tf);
    tf::poseTFToMsg(pose_camera_tf,pose_camera);
    //In questo punto ho T_0_camera


    Vector3d translation_camera_camera_gazebo(0,0,0);
    Matrix3d rotation_camera_camera_gazebo;
    Vector3d xvec_cc(0,0,1),yvec_cc(0,-1,0),zvec_cc(1,0,0);
    rotation_camera_camera_gazebo.col(0)=xvec_cc;
    rotation_camera_camera_gazebo.col(1)=yvec_cc;
    rotation_camera_camera_gazebo.col(2)=zvec_cc;

    T_camera_camera_gazebo.translation()=translation_camera_camera_gazebo;
    T_camera_camera_gazebo.linear()=rotation_camera_camera_gazebo;

    T_0_camera_gazebo=T_0_camera*T_camera_camera_gazebo;
    //In questo punto ho T_0_camera_gazebo


    Vector3d translation_camera_aruco(msg_from_bridge.x,msg_from_bridge.y,msg_from_bridge.z);
    Matrix3d rotation_camera_aruco;
    Vector3d xct(msg_from_bridge.vector[0],msg_from_bridge.vector[1],msg_from_bridge.vector[2]),yct(msg_from_bridge.vector[3],msg_from_bridge.vector[4],msg_from_bridge.vector[5]),zct(msg_from_bridge.vector[6],msg_from_bridge.vector[7],msg_from_bridge.vector[8]);
    rotation_camera_aruco.row(0)=xct;
    rotation_camera_aruco.row(1)=yct;
    rotation_camera_aruco.row(2)=zct;
    T_camera_aruco.translation()=translation_camera_aruco;
    T_camera_aruco.linear()=rotation_camera_aruco;

    T_0_aruco=T_0_camera_gazebo*T_camera_aruco;
    tf::poseEigenToTF(T_0_aruco,pose_aruco_tf);
    tf::poseTFToMsg(pose_aruco_tf,pose_aruco);
    //In questo punto ho T_0_aruco


    Matrix3d rotation_aruco_final_pos;
    Vector3d xaf(0,0,1),yaf(0,1,0),zaf(-1,0,0),trans_aruco_finalpos(0,0,0.22);//con il gripper che punta sull'aruco


    rotation_aruco_final_pos.row(0)=xaf;
    rotation_aruco_final_pos.row(1)=yaf;
    rotation_aruco_final_pos.row(2)=zaf;

    T_aruco_finalpos.linear()=rotation_aruco_final_pos;
    T_aruco_finalpos.translation()=trans_aruco_finalpos;

    T_0_finalpos=T_0_aruco*T_aruco_finalpos;
    //In questo punto ho T_0_final


    tf::poseEigenToTF(T_0_finalpos,pose_finalpos_tf);
    tf::poseTFToMsg(pose_finalpos_tf,pose_finalpos);


    //cout<<"T_0_final_pos translation:"<<endl<<T_tool_camera.translation()<<endl;
    //cout<<"T_0_final_pos rotation:"<<endl<<T_tool_camera.rotation()<<endl;

    cout<<"ee pose:"<<endl;
    stampa_Pose(pose_robot);

    cout<<"camera pose:"<<endl;
    stampa_Pose(pose_camera);

    cout<<"aruco pose:"<<endl;
    stampa_Pose(pose_aruco);

    cout<<"final pose:"<<endl;
    stampa_Pose(pose_finalpos);
    move_to_pose(pose_finalpos,true);
    robot->setPlanningTime(std_planning_time);
    return true;

  }
  else{
    ROS_INFO("No aruco detected, no return??");
    return false;
  }

}
