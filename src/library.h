#include <ros/ros.h>
#include "ros/service.h"
#include "ur3_control/aruco_service.h"
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
vector<double> joint_group_positions;
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
void take_object();
Pose pose_traslation_FAKE_solidale(Vector3d translation);
Pose pose_traslation_solidale(Vector3d translation);
void ruota_e_cerca_aruco();
void ruota_360();


void pick(string name_object){
  Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.2;
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
void ruota_360(){/*
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  joint_group_positions=robot->getCurrentJointValues();
  joint_group_positions[1] = grad_to_rad(-120);
  joint_group_positions[2] = grad_to_rad(100);
  joint_group_positions[3] = grad_to_rad(-100);
  joint_group_positions[4] = grad_to_rad(-90);




  double r,resolution,step,xc,yc,zc;
  double angle_resolution=1;
  for (int i= 0; i< 360/angle_resolution; i++)
    {

      joint_group_positions[0] = joint_group_positions[0]+grad_to_rad(angle_resolution);  // radians

      std::vector<double> joint_values;
      joint_values[0]=joint_group_positions[0];
      joint_values[1]=joint_group_positions[1];
      joint_values[2]=joint_group_positions[2];
      joint_values[3]=joint_group_positions[3];
      joint_values[4]=joint_group_positions[4];
      joint_values[5]=joint_group_positions[5];
      robot_state->setJointGroupPositions(joint_model_group, joint_values);
      const Eigen::Affine3d& link_pose = robot_state->getGlobalLinkTransform("link_name");
      Eigen::Vector3d cartesian_position = link_pose.translation();
      Eigen::Matrix3d link_orientation = link_pose.orientation();
    }






  vector<Pose> waypoints;

  Pose ee_point_goal;
  double r,resolution,step,xc,yc,zc;

  r=0.3;
  resolution=10;
  step=r/resolution;
  double angle_resolution=1/resolution;
  double d_angle;
  double angle= 0;
  d_angle = angle_resolution*3.14/180;

  xc=robot->getCurrentPose().pose.position.x;
  yc=robot->getCurrentPose().pose.position.y;
  zc=robot->getCurrentPose().pose.position.z;
  ee_point_goal.position.x=xc;
  ee_point_goal.orientation=robot->getCurrentPose().pose.orientation;

  for (int i= 0; i< 360/angle_resolution; i++)
    {
      angle+= d_angle;
      ee_point_goal.position.z = zc + r*cos(angle);
      ee_point_goal.position.y = yc + r*sin(angle);
      waypoints.push_back(ee_point_goal);
    }

      moveit_msgs::RobotTrajectory trajectory;
      robot->computeCartesianPath(waypoints,0.01,0,trajectory);

      success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
      my_plan.trajectory_=trajectory;
      sleep(1.0);
      robot->execute(my_plan);
      break;

*/
}
void ruota_e_cerca_aruco(){
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
  cout<<"Inserisci 'y' se vuoi che il robot arrivi con la testa bassa\nScelta:";
  cin>>scelta;
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

}
void trasformazioni(){
  Pose pose_robot=robot->getCurrentPose().pose;
  Affine3d T_actual;
  tf::Pose pose_robot_tf;
  tf::poseMsgToTF(pose_robot,pose_robot_tf);
  tf::poseTFToEigen(pose_robot_tf,T_actual);

  Matrix3d R_0_w3,R_0_cam,R_w3_cam,R_cam_aruco,R_0_aruco;
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
  Pose p;
  p.orientation.x=quat.getX();
  p.orientation.y=quat.getY();
  p.orientation.z=quat.getZ();
  p.orientation.w=quat.getW();
}
void reshape(){/*
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

/*

  T_actual.rotation

*/
