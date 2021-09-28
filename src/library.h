#include <signal.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "ros/service.h"
#include "ur3_control/aruco_service.h"
#include "ur3_control/UserInterface.h"
#include "ur3_control/collision_object_srv.h"
#include "ur3_control/float_return_srv.h"
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
#include "control_msgs/GripperCommandActionGoal.h"
#include <sstream>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include<list>

using namespace std;
using namespace geometry_msgs;
using namespace moveit;
using namespace planning_interface;
using namespace moveit_msgs;
using namespace Eigen;

#define ID_BUTTON_1 1
#define ID_BUTTON_2 2
#define ID_BUTTON_3 3
#define ID_BUTTON_4 4
#define ID_BUTTON_5 5
#define ID_BUTTON_6 6
#define ID_BUTTON_7 7
#define ID_BUTTON_8 8
#define ID_BUTTON_9 9
#define ID_IMU_MODULE 10
#define ID_IMU_DESTINATION_PLANE 11
#define ID_INSPECTION_WINDOW 12
#define ID_INSPECTION_WINDOW_COVER 13
#define ID_INSPECTION_WINDOW_COVER_STORAGE 14
#define aruco_length_array 15
#define pi 3.1415926535

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

int std_planning_time=7;
int aaaa;
ros::Publisher pub_gripper;
ros::Publisher pub_traj_cancel;

Affine3d T_tool_camera,T_camera_camera_gazebo,T_tool_camera_gazebo;

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
string str_rotaz_pannello="posizione_rotazione_ricerca_pannello";
string str_pos_iniziale="posizione_iniziale";
string str_pos_iniziale_cam_alta="str_pos_iniziale_cam_alta";
string str_pos_adatta_ricerca_per_terra="str_pos_adatta_ricerca_per_terra";
string posizione_gripper="open";

vector<double> pos_joint_home;
vector<double> pos_joint_up;
vector<double> pos_joint_r;
vector<double> pos_joint_centrifuga;
vector<double> pos_joint_pannello;
vector<double> pos_joint_iniziale;
vector<double> pos_joint_iniziale_cam_alta;
vector<double> pos_joint_adatta_ricerca_per_terra;
vector<double> debug;
vector<int> pos_rpy_tb;
vector<int> pos_rpy_ta;
bool show_log=false;

struct Affine_valid
{
     bool valid;
     Affine3d homo_matrix;
};
struct Pose_valid
{
     bool valid=false;
     Pose pose;
};
struct Collision_box_type{
  string name;
  PoseStamped pose;
  float size[3];
};

Pose_valid pose_pannello_elaborata;
Pose_valid Aruco_values[aruco_length_array];
map<string,Collision_box_type> collision_boxes;
//FLAGS
bool bool_exit=false;
bool adding_collision_enabled=false;
bool flagMiddlePanelCreated=false;
bool flagRightBoxCreated=false;
bool flagRightPanelCreated=false;
bool gazebo_bool=false;


void pick(string name_object,double distance_obj);
char getch();
double grad_to_rad(double grad);
void tf2quat_to_pose(tf2::Quaternion q,Pose *p);
void SetPoseOrientationRPY(Pose *p,int x0,int y0,int z0);
void AddToPoseOrientationRPYInGradi(Pose *p,double roll,double pitch,double yaw);
void ruotagiunto(unsigned int giunto,int angolo);
void set_angolo(unsigned int giunto,int angolo);
bool move_to_pose(geometry_msgs::Pose pt, bool Orientamento);
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
bool function_pose_aruco();
Pose homo_to_pose(Affine3d homo);
bool move_aruco_to_center_of_camera(double percentual_zoom);
bool ritorno_al_pannello(double percentual);
Affine_valid homo_0_aruco_elaration();
bool add_box(Collision_box_type box);
bool move_to_pose_optimized(geometry_msgs::Pose pose);
bool move_to_pose_cartesian(geometry_msgs::Pose pose);
Matrix3d from_rpy_to_rotational_matrix(double roll,double pitch,double yaw);
Affine3d pose_to_homo(Pose pose);
bool save_aruco_in_txt();
bool se_aruco_individuato_add_collision(int ID);




//OLD FUNCTIONS
char getch() {
//        char buf = 0;
//        struct termios old = {0};
//        if (tcgetattr(0, &old) < 0)
//                perror("tcsetattr()");
//        old.c_lflag &= ~ICANON;
//        old.c_lflag &= ~ECHO;
//        old.c_cc[VMIN] = 1;
//        old.c_cc[VTIME] = 0;
//        if (tcsetattr(0, TCSANOW, &old) < 0)
//                perror("tcsetattr ICANON");
//        if (read(0, &buf, 1) < 0)
//                perror ("read()");
//        old.c_lflag |= ICANON;
//        old.c_lflag |= ECHO;
//        if (tcsetattr(0, TCSADRAIN, &old) < 0)
//                perror ("tcsetattr ~ICANON");
//        return (buf);
}
void PosizioniBase_old(int posizione){
//  vector<double> joint_group_positions;
//  Pose target;
//  if(posizione<=2){
//    joint_group_positions=robot->getCurrentJointValues();
//    for(unsigned int i=0;i<6;i++)
//      joint_group_positions[i]=0;
//    if(posizione==2){
//      joint_group_positions[1]=grad_to_rad(-90);
//      joint_group_positions[4]=grad_to_rad(90);
//    }
//    robot->setJointValueTarget(joint_group_positions);
//  }
//  //testa in basso
//  if(posizione==3)
//  {
//    target.position=robot->getCurrentPose().pose.position;
//    SetPoseOrientationRPY(&target,0,-270,0);
//    robot->setPoseTarget(target);

//  }
//  //testa in alto
//  if(posizione==4)
//  {
//    target.position=robot->getCurrentPose().pose.position;
//    SetPoseOrientationRPY(&target,0,-90,0);
//    robot->setPoseTarget(target);

//  }
//  success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
//  //robot->move();
//  robot->move();

}
void take_object(){
//  tf2::Quaternion quat;
//  Pose target;
//  gazebo_msgs::GetModelState getmodelstate;
//  cout<<endl<<"nome oggetto: ";
//  cin>>nome_oggetto;
//  getmodelstate.request.model_name=nome_oggetto;
//  pose_object_client.call(getmodelstate);
//  cout<<endl<<"posizione di "<<nome_oggetto<<endl;
//  cout<<"X: "<<getmodelstate.response.pose.position.x<<endl;
//  cout<<"Y: "<<getmodelstate.response.pose.position.y<<endl;
//  cout<<"Z: "<<getmodelstate.response.pose.position.z<<endl;
//  cout<<endl<<"orientazione: "<<endl;
//  cout<<"X: "<<getmodelstate.response.pose.orientation.x<<endl;
//  cout<<"Y: "<<getmodelstate.response.pose.orientation.y<<endl;
//  cout<<"Z: "<<getmodelstate.response.pose.orientation.z<<endl;
//  cout<<"W: "<<getmodelstate.response.pose.orientation.w<<endl;

// //PosizioniBase(2);
// //usleep(5000000);
//  target.position.x=getmodelstate.response.pose.position.x;
//  target.position.y=getmodelstate.response.pose.position.y;
//  target.position.z=getmodelstate.response.pose.position.z+0.05;
//  quat.setRPY(grad_to_rad(0),grad_to_rad(90),grad_to_rad(0));
//  tf2quat_to_pose(quat,&target);
//  move_to_pose(target,true);
}
Pose pose_traslation_FAKE_solidale(Vector3d translation)
{
  Pose pose_robot=robot->getCurrentPose().pose;
  /*tf2::Quaternion quat1;
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
  pose_robot.orientation.w=quat2.getW();*/

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
//  ROS_INFO("Starting rotating 360");




//  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

//  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
//  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
////const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
//  ROS_INFO("kinematic model aquired");

//  //rotazione

//  vector<double> joint_positions;
//  joint_positions=robot->getCurrentJointValues();



//  double angle_resolution=1;
//  vector<Pose> waypoints;
//  for (int i= 0; i< 360/angle_resolution; i++)
//    {
//      joint_positions[0] = joint_positions[0]+grad_to_rad(angle_resolution);  // radians


//      kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);


//      tf::Pose target_tf;
//      Pose target;
//      const Eigen::Affine3d& link_pose = kinematic_state->getGlobalLinkTransform("tool0");//forse tool0
//      tf::poseEigenToTF(link_pose,target_tf);
//      tf::poseTFToMsg(target_tf,target);
//      SetPoseOrientationRPY(&target,0,90,0);
//      waypoints.push_back(target);
//    }

//      moveit_msgs::RobotTrajectory trajectory;
//      robot->computeCartesianPath(waypoints,0.0001,0,trajectory);

//      success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
//      ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
//      my_plan.trajectory_=trajectory;
//      robot->execute(my_plan);

//      ROS_INFO("Rotazione 360 completata");

}
void trasformazioni(){
//  Pose pose_robot=robot->getCurrentPose().pose;

//  ROS_INFO("Traslazione:");
//  ROS_INFO("x: %f\n y:%f\n z:%f\n ",pose_robot.position.x,pose_robot.position.y,pose_robot.position.z);

//  ROS_INFO("Rotazione:");
//  ROS_INFO("x: %f\n y:%f\n z:%f\n w:%f\n",pose_robot.orientation.x,pose_robot.orientation.y,pose_robot.orientation.z,pose_robot.orientation.w);

//  Pose pose_robot_target;
//  Affine3d T_0_tool,T_tool_camera,T_0_camera;
//  tf::Pose pose_robot_tf,pose_target_tf;
//  tf::poseMsgToTF(pose_robot,pose_robot_tf);
//  tf::poseTFToEigen(pose_robot_tf,T_0_tool);

//  cout<<"T_0_tool translation:"<<endl<<T_0_tool.translation()<<endl;
//  cout<<"T_0_tool rotation:"<<endl<<T_0_tool.rotation()<<endl;

//  /*
//   *
//   *    tra ee_link e camera_realsense_gazebo
//        A=np.array([[0,1,0,0.4569],[1,0,0,0.19425],[0,0,-1,0.6655],[0,0,0,1]])
//        B=np.array([[0,0,1,0.025],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
//        p=np.array([[0],[0],[0],[1]])
//        T=A.dot(B)
//        result=T.dot(p)
//*/
//  Vector3d translation_tool_camera(0.025,0,0);
//  Matrix3d rotation_tool_camera;
//  Vector3d xvec_des(0,1,0),yvec_des(0,0,1),zvec_des(1,0,0);
//  rotation_tool_camera.col(0)=xvec_des;
//  rotation_tool_camera.col(1)=yvec_des;
//  rotation_tool_camera.col(2)=zvec_des;


//  T_tool_camera.translation()=translation_tool_camera;
//  T_tool_camera.linear()=rotation_tool_camera;
//  cout<<"T_tool_camera translation:"<<endl<<T_tool_camera.translation()<<endl;
//  cout<<"T_tool_camera rotation:"<<endl<<T_tool_camera.rotation()<<endl;
//  //ROS_INFO("T_tool_camera: %f %f %f",T_tool_camera.translation().x(),T_tool_camera.translation().y(),T_tool_camera.translation().z());


//  T_0_camera=T_0_tool*T_tool_camera;

//  tf::poseEigenToTF(T_0_camera,pose_target_tf);
//  tf::poseTFToMsg(pose_target_tf,pose_robot_target);

//  ROS_INFO("Traslazione:");
//  ROS_INFO("x: %f\n y:%f\n z:%f\n ",pose_robot_target.position.x,pose_robot_target.position.y,pose_robot_target.position.z);

//  ROS_INFO("Rotazione:");
//  ROS_INFO("x: %f\n y:%f\n z:%f\n w:%f\n",pose_robot_target.orientation.x,pose_robot_target.orientation.y,pose_robot_target.orientation.z,pose_robot_target.orientation.w);

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

//  /*Non funzionante*/

//  bridge_service(str_md_bpa,"");

//  PosizioniBase(str_r);//si mette in posizione prima di iniziare a ruotare

//  if(!move_to_aruco())//controlla se nell'attuale posizione c'è un aruco e nel caso lo piglia
//  {
//    ruota_360();
//    if(!move_to_aruco())//controlla se nell'attuale posizione c'è un aruco e nel caso lo piglia
//    {
//      ROS_INFO("Aruco non trovato :(");
//    }
//  }

//  bridge_service(str_md_stop_bpa,"");
}
bool move_to_aruco(){
//  Pose aruco_pose_solidale;
//  if(individua_aruco(&aruco_pose_solidale)){

//    robot->setPlanningTime(10);
//    move_to_pose(aruco_pose_solidale,true);
//    robot->setPlanningTime(std_planning_time);
//    return true;
//  }
//  else {
//    return false;
//  }
}
void move_near_to_centrifuga(){


// PosizioniBase(str_r);
// PosizioniBase(str_near_centrifuga);



}
void move_to_centrifuga(){

//  move_near_to_centrifuga();
//  PosizioniBase(str_centrifuga);

}
bool move_to_pannello_aruco(){

//  Pose aruco_pose_solidale;
//  if(individua_aruco(&aruco_pose_solidale)){

//    robot->setPlanningTime(10);
//    move_to_pose(aruco_pose_solidale,true);
//    robot->setPlanningTime(std_planning_time);
//    return true;
//  }
//  else {
//    return false;
//  }
}
void aruco_pannello(){
//  bool continua_=true;
//  Pose temp;

//  while(continua_){

//    PosizioniBase(str_pannello);
    
//    function_pose_aruco();

//    if(bridge_service(str_md_next_aruco,"").moreTargets<=0){
//        continua_=false;
//    }

//  }
////muoviti sul primo aruco
////ritorna in un buon punto
////cambia aruco. se non c'è next esci

}
bool individua_aruco(Pose *aruco_pose_solidale){
////
//  ur3_control::aruco_serviceResponse aruco_srv_msg_resp=bridge_service(str_md_rd,"");
//  ROS_INFO("Aruco found:%s",(aruco_srv_msg_resp.aruco_found ? "YES":"NO"));

//  if(aruco_srv_msg_resp.aruco_found){
//    return true;
//  }
//  else{
//    return false;
//  }
ROS_INFO("BROKEEN");
return false;
}



//ACTUAL FUNCTIONS


//Service
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
ur3_control::collision_object_srvResponse collision_service(ur3_control::collision_object_srvRequest coll_req){
  ros::NodeHandle node_handle;
  ros::ServiceClient client1;
  ur3_control::collision_object_srv msg;
  client1 = node_handle.serviceClient<ur3_control::collision_object_srv>("/collision_server");

  msg.request=coll_req;
  client1.call(msg);
  return msg.response;
}


//Void comode

bool cambia_aruco(string ID){
  for(int j=0;j<10;j++){
    ur3_control::aruco_serviceResponse msg=bridge_service(str_md_next_aruco,ID);
    if(std::to_string(msg.id_aruco)==ID){
      return true;
    }

    uint milliseconds=500;
    for(int i=0;i<10;i++){
      usleep(milliseconds*1000);//from milliseconds to microseconds
      msg=bridge_service(str_md_next_aruco,ID);
      if(std::to_string(msg.id_aruco)==ID){
        return true;
      }
    }
  }
  return false;
}
bool aruco_individuato(){

  return bridge_service(str_md_rd,"").aruco_found;

}
bool aggiorna_aruco_selezionato(int ID){
  ur3_control::aruco_serviceResponse msg;
  msg=bridge_service(str_md_rd,"");




  if(!msg.all_aruco_found[ID]==true){
    return false;
  }

  cambia_aruco(to_string(ID));


  if(aruco_individuato()) {
    ROS_INFO("ARUCO: %d INDIVIDUATO",ID);
    Affine_valid T_0_aruco_valid=homo_0_aruco_elaration();

    if(T_0_aruco_valid.valid){

      Aruco_values[ID].pose=homo_to_pose(T_0_aruco_valid.homo_matrix);
      Aruco_values[ID].valid=true;
      se_aruco_individuato_add_collision(ID);

      save_aruco_in_txt();
    }
    else{
      return false;
    }
  }




  return true;
}
bool se_aruco_individuato_add_collision(int ID){
  //Creazione middle panel
  if((ID==ID_BUTTON_2 ||ID==ID_BUTTON_5 ||ID==ID_BUTTON_8)
     && !(flagMiddlePanelCreated)){

    flagMiddlePanelCreated=true;
    PoseStamped box_pose;
    float box_size[3];
    string box_name="middle_panel";

    box_size[0]=0.001;//thickness
    box_size[1]=0.3;//width
    box_size[2]=3*0.5;//height

    box_pose.header.frame_id="base_link";


    SetPoseOrientationRPY(&box_pose.pose,0,0,0);
    //no orientation change wrt base frame

    box_pose.pose.position.x=Aruco_values[ID].pose.position.x+box_size[0]/2;//sicurezza
    box_pose.pose.position.y=Aruco_values[ID].pose.position.y;
    box_pose.pose.position.z=Aruco_values[ID].pose.position.z;

    collision_boxes[box_name].name=box_name;
    collision_boxes[box_name].pose=box_pose;
    collision_boxes[box_name].size[0]=box_size[0];
    collision_boxes[box_name].size[1]=box_size[1];
    collision_boxes[box_name].size[2]=box_size[2];
    add_box(collision_boxes[box_name]);


  }

  if(ID>=ID_BUTTON_1 && ID<=ID_BUTTON_9){
    Affine3d T_aruco_finalpos;
    T_aruco_finalpos.translation().x()=0;
    T_aruco_finalpos.translation().y()=-0.055;
    T_aruco_finalpos.translation().z()=0;
    T_aruco_finalpos.linear()=from_rpy_to_rotational_matrix(0,0,0);//non mi interessa cambiare la rotazione


    Affine3d T_0_finalpos_premuto=pose_to_homo(Aruco_values[ID].pose)*T_aruco_finalpos;
    Pose pose_pulsante=homo_to_pose(T_0_finalpos_premuto);

    PoseStamped box_pose;
    float box_size[3];
    string box_name="button_"+to_string(ID);

    box_size[0]=0.023;//thickness
    box_size[1]=0.04;//width
    box_size[2]=0.04;//height

    box_pose.header.frame_id="base_link";


    SetPoseOrientationRPY(&box_pose.pose,0,0,0);
    //no orientation change wrt base frame

    box_pose.pose.position.x=pose_pulsante.position.x-box_size[0]/2;//sicurezza
    box_pose.pose.position.y=pose_pulsante.position.y;
    box_pose.pose.position.z=pose_pulsante.position.z;

    collision_boxes[box_name].name=box_name;
    collision_boxes[box_name].pose=box_pose;
    collision_boxes[box_name].size[0]=box_size[0];
    collision_boxes[box_name].size[1]=box_size[1];
    collision_boxes[box_name].size[2]=box_size[2];


    add_box(collision_boxes[box_name]);

  }

  if((ID==ID_INSPECTION_WINDOW_COVER || ID==ID_INSPECTION_WINDOW) && !(flagRightBoxCreated)){

    flagRightBoxCreated=true;
    Affine3d T_aruco_boxcenter,T_0_rightborder,T_0_leftborder,T_0_belowborder,T_0_upperborder,T_0_fondobox,T_0_coperchio,T_0_pomello;
    Affine3d T_boxcenter_rightborder,T_boxcenter_leftborder,T_boxcenter_upperborder,T_boxcenter_belowborder,T_boxcenter_fondobox,T_boxcenter_coperchio,T_boxcenter_pomello;
    Pose pose_rightborder,pose_leftborder,pose_belowborder,pose_upperborder,pose_fondobox,pose_pomello,pose_coperchio;

    PoseStamped box_pose_rightborder,box_pose_leftborder,box_pose_upperborder,box_pose_belowborder,box_pose_fondobox,box_pose_coperchio,box_pose_pomello;
    float box_size_right_border[3],box_size_below_border[3],box_size_upper_border[3],box_size_left_border[3],box_size_fondobox[3],box_size_coperchio[3],box_size_pomello[3];
    string box_name_rightborder,box_name_leftborder,box_name_upperborder,box_name_belowborder,box_name_fondobox,box_name_coperchio,box_name_pomello;


    if(ID==ID_INSPECTION_WINDOW_COVER){

    T_aruco_boxcenter.translation().x()=0.0499;
    T_aruco_boxcenter.translation().y()=0.0249;
    T_aruco_boxcenter.translation().z()=-(0.07/2);
    T_aruco_boxcenter.linear()=from_rpy_to_rotational_matrix(0,0,0);

    }
    else if(ID==ID_INSPECTION_WINDOW){

      T_aruco_boxcenter.translation().x()=0;
      T_aruco_boxcenter.translation().y()=0;
      T_aruco_boxcenter.translation().z()=-(0.1/2);
      T_aruco_boxcenter.linear()=from_rpy_to_rotational_matrix(-M_PI/2,0,0);


    }
    else{
      ROS_INFO("Questo errore non dovrebbe esistere");
      return false;
    }

    T_boxcenter_leftborder.translation().x()=-(0.15)/2;
    T_boxcenter_leftborder.translation().y()=0;
    T_boxcenter_leftborder.translation().z()=0;
    T_boxcenter_leftborder.linear()=from_rpy_to_rotational_matrix(0,0,0);


    T_boxcenter_rightborder.translation().x()=(0.15)/2;//cambia solo questo
    T_boxcenter_rightborder.translation().y()=0;
    T_boxcenter_rightborder.translation().z()=0;
    T_boxcenter_rightborder.linear()=from_rpy_to_rotational_matrix(0,0,0);


    T_boxcenter_upperborder.translation().x()=0;
    T_boxcenter_upperborder.translation().y()=(0.1/2);
    T_boxcenter_upperborder.translation().z()=0;
    T_boxcenter_upperborder.linear()=from_rpy_to_rotational_matrix(0,0,0);


    T_boxcenter_belowborder.translation().x()=0;
    T_boxcenter_belowborder.translation().y()=-(0.1/2);
    T_boxcenter_belowborder.translation().z()=0;
    T_boxcenter_belowborder.linear()=from_rpy_to_rotational_matrix(0,0,0);

    T_boxcenter_fondobox.translation().x()=0;
    T_boxcenter_fondobox.translation().y()=0;
    T_boxcenter_fondobox.translation().z()=-(0.07)/2;
    T_boxcenter_fondobox.linear()=from_rpy_to_rotational_matrix(0,0,0);

    T_boxcenter_coperchio.translation().x()=0;
    T_boxcenter_coperchio.translation().y()=0;
    T_boxcenter_coperchio.translation().z()=(0.07)/2;
    T_boxcenter_coperchio.linear()=from_rpy_to_rotational_matrix(0,0,0);

    T_boxcenter_pomello.translation().x()=0;
    T_boxcenter_pomello.translation().y()=0;
    T_boxcenter_pomello.translation().z()=((0.07)/2 + 0.035/2);
    T_boxcenter_pomello.linear()=from_rpy_to_rotational_matrix(0,0,0);




    T_0_leftborder=pose_to_homo(Aruco_values[ID].pose)*T_aruco_boxcenter*T_boxcenter_leftborder;
    pose_leftborder=homo_to_pose(T_0_leftborder);


    T_0_rightborder=pose_to_homo(Aruco_values[ID].pose)*T_aruco_boxcenter*T_boxcenter_rightborder;
    pose_rightborder=homo_to_pose(T_0_rightborder);


    T_0_upperborder=pose_to_homo(Aruco_values[ID].pose)*T_aruco_boxcenter*T_boxcenter_upperborder;
    pose_upperborder=homo_to_pose(T_0_upperborder);


    T_0_belowborder=pose_to_homo(Aruco_values[ID].pose)*T_aruco_boxcenter*T_boxcenter_belowborder;
    pose_belowborder=homo_to_pose(T_0_belowborder);

    T_0_fondobox=pose_to_homo(Aruco_values[ID].pose)*T_aruco_boxcenter*T_boxcenter_fondobox;
    pose_fondobox=homo_to_pose(T_0_fondobox);

    T_0_coperchio=pose_to_homo(Aruco_values[ID].pose)*T_aruco_boxcenter*T_boxcenter_coperchio;
    pose_coperchio=homo_to_pose(T_0_coperchio);

    T_0_pomello=pose_to_homo(Aruco_values[ID].pose)*T_aruco_boxcenter*T_boxcenter_pomello;
    pose_pomello=homo_to_pose(T_0_pomello);



    box_name_leftborder="inspection_box_leftborder";
    box_name_rightborder="inspection_box_rightborder";
    box_name_upperborder="inspection_box_upperborder";
    box_name_belowborder="inspection_box_belowborder";
    box_name_fondobox="inspection_box_fondobox";
    box_name_coperchio="inspection_box_base_coperchio";
    box_name_pomello="inspection_box_pomello_coperchio";


    box_size_left_border[0]=0.001;//thickness
    box_size_left_border[1]=0.1;//width
    box_size_left_border[2]=0.07;//height

    box_size_right_border[0]=0.001;//thickness
    box_size_right_border[1]=0.1;//width
    box_size_right_border[2]=0.07;//height

    box_size_below_border[0]=0.15;//thickness
    box_size_below_border[1]=0.001;//width
    box_size_below_border[2]=0.07;//height

    box_size_upper_border[0]=0.15;//thickness
    box_size_upper_border[1]=0.001;//width
    box_size_upper_border[2]=0.07;//height

    box_size_fondobox[0]=0.15;//thickness
    box_size_fondobox[1]=0.1;//width
    box_size_fondobox[2]=0.001;//height

    box_size_coperchio[0]=0.15;//thickness
    box_size_coperchio[1]=0.1;//width
    box_size_coperchio[2]=0.001;//height

    box_size_pomello[0]=0.035;//thickness
    box_size_pomello[1]=0.035;//width
    box_size_pomello[2]=0.035;//height



    box_pose_leftborder.header.frame_id="base_link";
    box_pose_leftborder.pose.orientation=pose_leftborder.orientation;
    box_pose_leftborder.pose.position.x=pose_leftborder.position.x;
    box_pose_leftborder.pose.position.y=pose_leftborder.position.y;
    box_pose_leftborder.pose.position.z=pose_leftborder.position.z;

    box_pose_rightborder.header.frame_id="base_link";
    box_pose_rightborder.pose.orientation=pose_rightborder.orientation;
    box_pose_rightborder.pose.position.x=pose_rightborder.position.x;
    box_pose_rightborder.pose.position.y=pose_rightborder.position.y;
    box_pose_rightborder.pose.position.z=pose_rightborder.position.z;

    box_pose_upperborder.header.frame_id="base_link";
    box_pose_upperborder.pose.orientation=pose_upperborder.orientation;
    box_pose_upperborder.pose.position.x=pose_upperborder.position.x;
    box_pose_upperborder.pose.position.y=pose_upperborder.position.y;
    box_pose_upperborder.pose.position.z=pose_upperborder.position.z;

    box_pose_belowborder.header.frame_id="base_link";
    box_pose_belowborder.pose.orientation=pose_belowborder.orientation;
    box_pose_belowborder.pose.position.x=pose_belowborder.position.x;
    box_pose_belowborder.pose.position.y=pose_belowborder.position.y;
    box_pose_belowborder.pose.position.z=pose_belowborder.position.z;


    box_pose_fondobox.header.frame_id="base_link";
    box_pose_fondobox.pose.orientation=pose_fondobox.orientation;
    box_pose_fondobox.pose.position.x=pose_fondobox.position.x;
    box_pose_fondobox.pose.position.y=pose_fondobox.position.y;
    box_pose_fondobox.pose.position.z=pose_fondobox.position.z;

    box_pose_coperchio.pose=pose_coperchio;
    box_pose_coperchio.header.frame_id="base_link";

    box_pose_pomello.pose=pose_pomello;
    box_pose_pomello.header.frame_id="base_link";





//    add_box(box_name_leftborder,box_pose_leftborder,box_size_left_border);
//    add_box(box_name_rightborder,box_pose_rightborder,box_size_right_border);
//    add_box(box_name_upperborder,box_pose_upperborder,box_size_upper_border);
//    add_box(box_name_belowborder,box_pose_belowborder,box_size_below_border);
//    add_box(box_name_fondobox,box_pose_fondobox,box_size_fondobox);
//    add_box(box_name_coperchio,box_pose_coperchio,box_size_coperchio);
//    add_box(box_name_pomello,box_pose_pomello,box_size_pomello);

    collision_boxes[box_name_leftborder].name=box_name_leftborder;
    collision_boxes[box_name_leftborder].pose=box_pose_leftborder;
    collision_boxes[box_name_leftborder].size[0]=box_size_left_border[0];
    collision_boxes[box_name_leftborder].size[1]=box_size_left_border[1];
    collision_boxes[box_name_leftborder].size[2]=box_size_left_border[2];
    add_box(collision_boxes[box_name_leftborder]);

    collision_boxes[box_name_rightborder].name=box_name_rightborder;
    collision_boxes[box_name_rightborder].pose=box_pose_rightborder;
    collision_boxes[box_name_rightborder].size[0]=box_size_right_border[0];
    collision_boxes[box_name_rightborder].size[1]=box_size_right_border[1];
    collision_boxes[box_name_rightborder].size[2]=box_size_right_border[2];
    add_box(collision_boxes[box_name_rightborder]);


    collision_boxes[box_name_upperborder].name=box_name_upperborder;
    collision_boxes[box_name_upperborder].pose=box_pose_upperborder;
    collision_boxes[box_name_upperborder].size[0]=box_size_upper_border[0];
    collision_boxes[box_name_upperborder].size[1]=box_size_upper_border[1];
    collision_boxes[box_name_upperborder].size[2]=box_size_upper_border[2];
    add_box(collision_boxes[box_name_upperborder]);


    collision_boxes[box_name_belowborder].name=box_name_belowborder;
    collision_boxes[box_name_belowborder].pose=box_pose_belowborder;
    collision_boxes[box_name_belowborder].size[0]=box_size_below_border[0];
    collision_boxes[box_name_belowborder].size[1]=box_size_below_border[1];
    collision_boxes[box_name_belowborder].size[2]=box_size_below_border[2];
    add_box(collision_boxes[box_name_belowborder]);


    collision_boxes[box_name_fondobox].name=box_name_fondobox;
    collision_boxes[box_name_fondobox].pose=box_pose_fondobox;
    collision_boxes[box_name_fondobox].size[0]=box_size_fondobox[0];
    collision_boxes[box_name_fondobox].size[1]=box_size_fondobox[1];
    collision_boxes[box_name_fondobox].size[2]=box_size_fondobox[2];
    add_box(collision_boxes[box_name_fondobox]);


    collision_boxes[box_name_pomello].name=box_name_pomello;
    collision_boxes[box_name_pomello].pose=box_pose_pomello;
    collision_boxes[box_name_pomello].size[0]=box_size_pomello[0];
    collision_boxes[box_name_pomello].size[1]=box_size_pomello[1];
    collision_boxes[box_name_pomello].size[2]=box_size_pomello[2];
    add_box(collision_boxes[box_name_pomello]);



    collision_boxes[box_name_coperchio].name=box_name_coperchio;
    collision_boxes[box_name_coperchio].pose=box_pose_coperchio;
    collision_boxes[box_name_coperchio].size[0]=box_size_coperchio[0];
    collision_boxes[box_name_coperchio].size[1]=box_size_coperchio[1];
    collision_boxes[box_name_coperchio].size[2]=box_size_coperchio[2];
    add_box(collision_boxes[box_name_coperchio]);






  }
  
  if((ID==ID_INSPECTION_WINDOW_COVER || ID==ID_INSPECTION_WINDOW) && !(flagRightPanelCreated)){
    flagRightPanelCreated=true;
    Affine3d T_aruco_rightpanel,T_0_rightpanel;
    Pose pose_rightpanel;

    PoseStamped box_pose_rightpanel;
    float box_size_rightpanel[3];
    string box_name_rightpanel;

    if(ID==ID_INSPECTION_WINDOW_COVER){

    T_aruco_rightpanel.translation().x()=0;//assumo che sia nel centro
    T_aruco_rightpanel.translation().y()=0.0249+0.0996/2;
    T_aruco_rightpanel.translation().z()=-(0.18/2);
    T_aruco_rightpanel.linear()=from_rpy_to_rotational_matrix(0,0,0);

    }
    else if(ID==ID_INSPECTION_WINDOW){

      T_aruco_rightpanel.translation().x()=-(0.1496/2 - 0.0049 - 0.04/2);
      T_aruco_rightpanel.translation().y()=(0.07/2 - 0.18/2);
      T_aruco_rightpanel.translation().z()=-(0.0996);
      T_aruco_rightpanel.linear()=from_rpy_to_rotational_matrix(-M_PI/2,0,0);


    }
    else{
      ROS_INFO("Questo errore non dovrebbe esistere");
      return false;
    }

    T_0_rightpanel=pose_to_homo(Aruco_values[ID].pose)*T_aruco_rightpanel;
    pose_rightpanel=homo_to_pose(T_0_rightpanel);

    box_name_rightpanel="right_panel";

    box_size_rightpanel[0]=0.25+0.05;//thickness HO AGGIUNTO 5 cm di sicurezza
    box_size_rightpanel[1]=0.001;//width
    box_size_rightpanel[2]=0.18;//height

    box_pose_rightpanel.pose=pose_rightpanel;
    box_pose_rightpanel.header.frame_id="base_link";



    collision_boxes[box_name_rightpanel].name=box_name_rightpanel;
    collision_boxes[box_name_rightpanel].pose=box_pose_rightpanel;
    collision_boxes[box_name_rightpanel].size[0]=box_size_rightpanel[0];
    collision_boxes[box_name_rightpanel].size[1]=box_size_rightpanel[1];
    collision_boxes[box_name_rightpanel].size[2]=box_size_rightpanel[2];
    add_box(collision_boxes[box_name_rightpanel]);

}

  if(ID==ID_IMU_DESTINATION_PLANE ){
    Affine3d T_aruco_leftpanel,T_0_leftpanel;
    Pose pose_leftpanel;

    PoseStamped box_pose_leftpanel;
    float box_size_leftpanel[3];
    string box_name_leftpanel;


    T_aruco_leftpanel.translation().x()=(-0.06/2 - 0.025 +0.24/2);//assumo che sia nel centro
    T_aruco_leftpanel.translation().y()=(0.05/2 - 0.35/2);
    T_aruco_leftpanel.translation().z()=0;
    T_aruco_leftpanel.linear()=from_rpy_to_rotational_matrix(0,0,0);


    T_0_leftpanel=pose_to_homo(Aruco_values[ID].pose)*T_aruco_leftpanel;
    pose_leftpanel=homo_to_pose(T_0_leftpanel);

    box_name_leftpanel="left_panel";

    box_size_leftpanel[0]=0.25+0.05;//thickness HO AGGIUNTO MARGINE
    box_size_leftpanel[1]=0.36;//width
    box_size_leftpanel[2]=0.001;//height

    box_pose_leftpanel.pose=pose_leftpanel;
    box_pose_leftpanel.header.frame_id="base_link";

    collision_boxes[box_name_leftpanel].name=box_name_leftpanel;
    collision_boxes[box_name_leftpanel].pose=box_pose_leftpanel;
    collision_boxes[box_name_leftpanel].size[0]=box_size_leftpanel[0];
    collision_boxes[box_name_leftpanel].size[1]=box_size_leftpanel[1];
    collision_boxes[box_name_leftpanel].size[2]=box_size_leftpanel[2];
    add_box(collision_boxes[box_name_leftpanel]);




}

  if(ID==ID_IMU_MODULE){
    Affine3d T_aruco_imu,T_0_imu;
    Pose pose_imu;

    string imu_name="imu_module";

    T_aruco_imu.translation().x()=0;//assumo che sia nel centro
    T_aruco_imu.translation().y()=0;
    T_aruco_imu.translation().z()=-0.0262;//1.2mm +25mm=26.2mm
    T_aruco_imu.linear()=from_rpy_to_rotational_matrix(0,0,0);


    T_0_imu=pose_to_homo(Aruco_values[ID].pose)*T_aruco_imu;
    pose_imu=homo_to_pose(T_0_imu);

    collision_boxes[imu_name].name=imu_name;
    collision_boxes[imu_name].size[0]=0.165;//thickness HO AGGIUNTO MARGINE
    collision_boxes[imu_name].size[1]=0.05;//width
    collision_boxes[imu_name].size[2]=0.052;//height

    collision_boxes[imu_name].pose.pose=pose_imu;
    collision_boxes[imu_name].pose.header.frame_id="base_link";

    add_box(collision_boxes[imu_name]);

  }
  return true;
}
bool se_aruco_individuato_aggiorna_array(int ID){

  //se chiamata con ID==0 controllerà tutti gli ID e se ne avrà trovato almeno uno restituirà true

  ur3_control::aruco_serviceResponse msg;
  msg=bridge_service(str_md_rd,"");


  bool aruco_di_interesse_aggiornato=false;
  bool almeno_uno_trovato=false;


  for(int i=0;i<msg.all_aruco_found.size();i++){
    if(msg.all_aruco_found[i]==true && !Aruco_values[i].valid){
      almeno_uno_trovato=true;
    }
  }

  if(!almeno_uno_trovato)
    return false;

  sleep(2);

  msg=bridge_service(str_md_rd,"");

  for(int i=0;i<msg.all_aruco_found.size();i++){
    if(msg.all_aruco_found[i]==true){
      cambia_aruco(to_string(i));


      if(aruco_individuato()) {
        ROS_INFO("ARUCO: %d INDIVIDUATO",i);
        Affine_valid T_0_aruco_valid=homo_0_aruco_elaration();

        if(T_0_aruco_valid.valid){

          if(i==ID)
            aruco_di_interesse_aggiornato=true;

          Aruco_values[i].pose=homo_to_pose(T_0_aruco_valid.homo_matrix);

          //Aggiungo box di collision se non è mai stato trovato
          if(!Aruco_values[i].valid){
            ROS_INFO("E' la prima volta che vedo questo aruco");
            Aruco_values[i].valid=true;
            se_aruco_individuato_add_collision(i);

          }
          Aruco_values[i].valid=true;


        }




      }
    }

  }
  cambia_aruco(to_string(ID));
  sleep(2);

  save_aruco_in_txt();

  if(aruco_di_interesse_aggiornato)
    return true;
  if(ID==0){
    return true;
  }

  return false;
}
void controlla(char key){
    Vector3d translation(0,0,0);
    string k;
    Pose targ,pose_robot=robot->getCurrentPose().pose;
    targ=pose_robot;
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
    case 'h':{



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
    }



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
        robot->setPlanningTime(0.5);
        move_to_pose_cartesian(targ);
        robot->setPlanningTime(std_planning_time);
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
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   // Terminate program
   bool_exit=true;
   //exit(signum);
}
void exit_from_all(){

  ur3_control::collision_object_srvRequest coll_req;
  coll_req.exit=true;
  coll_req.add=false;

  collision_service(coll_req);

  bridge_service("exit","");

  bool_exit=true;

}
bool add_box(Collision_box_type box){

  if(!adding_collision_enabled){
    return false;
  }

  ur3_control::collision_object_srvRequest coll_srv;
  coll_srv.add=true;
  coll_srv.exit=false;
  coll_srv.box_name=box.name;
  coll_srv.box_pose=box.pose;
  coll_srv.box_size.push_back(box.size[0]);
  coll_srv.box_size.push_back(box.size[1]);
  coll_srv.box_size.push_back(box.size[2]);


  return collision_service(coll_srv).success;

}
bool remove_box(string box_name){
  ROS_INFO("REMOVING BOX");
  ur3_control::collision_object_srvRequest coll_srv;

  coll_srv.add=false;
  coll_srv.box_name=box_name;

  return collision_service(coll_srv).success;

}
bool attach_box(Collision_box_type box){

  if(!adding_collision_enabled){
    return false;
  }

  ur3_control::collision_object_srvRequest coll_srv;
  coll_srv.add=false;
  coll_srv.exit=false;
  coll_srv.attach=true;
  coll_srv.box_name=box.name;
  coll_srv.box_pose=box.pose;
  coll_srv.box_size.push_back(box.size[0]);
  coll_srv.box_size.push_back(box.size[1]);
  coll_srv.box_size.push_back(box.size[2]);


  return collision_service(coll_srv).success;

}
bool detach_box(Collision_box_type box){

  if(!adding_collision_enabled){
    return false;
  }

  ur3_control::collision_object_srvRequest coll_srv;
  coll_srv.add=false;
  coll_srv.exit=false;
  coll_srv.attach=false;
  coll_srv.detach=true;
  coll_srv.box_name=box.name;
  coll_srv.box_pose=box.pose;
  coll_srv.box_size.push_back(box.size[0]);
  coll_srv.box_size.push_back(box.size[1]);
  coll_srv.box_size.push_back(box.size[2]);


  return collision_service(coll_srv).success;

}
bool add_and_attach_box(Collision_box_type box){
  add_box(box);
  attach_box(box);
}
bool remove_and_detach_box(Collision_box_type box){
  detach_box(box);
  remove_box(box.name);
}
void blocca_se_vedi_nuovo_aruco(){
  ros::NodeHandle node_handle;
  ros::ServiceClient client1;
  client1 = node_handle.serviceClient<ur3_control::aruco_service>("/aruco_modality");
  ur3_control::aruco_service aruco_srv_msg;

  aruco_srv_msg.request.modality="blocca_se_vedi_nuovo_aruco";
  aruco_srv_msg.request.second_information="";

  bool aruco_da_trovare[aruco_length_array];
  aruco_srv_msg.request.aruco_trovati.push_back(true);
  for(int i=1;i<aruco_length_array;i++){
    aruco_srv_msg.request.aruco_trovati.push_back(Aruco_values[i].valid);
  }
  client1.call(aruco_srv_msg);
}
void smetti_di_bloccare_se_vedi_nuovo_aruco(){
  ros::NodeHandle node_handle;
  ros::ServiceClient client1;
  client1 = node_handle.serviceClient<ur3_control::aruco_service>("/aruco_modality");
  ur3_control::aruco_service aruco_srv_msg;

  aruco_srv_msg.request.modality="smetti_di_bloccare_se_vedi_nuovo_aruco";
  aruco_srv_msg.request.second_information="";


  client1.call(aruco_srv_msg);
}
bool save_aruco_in_txt(){
  ofstream outFile;
  string nome,tipo;
  string pkgpath = ros::package::getPath("ur3_control");
  string path_txt="/txt/aruco_values.txt";
  string path_total=pkgpath + path_txt;
  if(show_log)
    cout<<"percorso txt: "<<path_total<<endl<<endl;
  ROS_INFO("Aggiorno txt aruco pos");
  outFile.open(path_total);
  if (!outFile) {
      cerr << "Unable to open file datafile.txt";
      return false;  // call system to stop
  }
  for(int i=1;i<aruco_length_array;i++){

    outFile.write((char*) &Aruco_values[i],sizeof(Aruco_values[i]));

  }


  outFile.close();
  return true;
}
void pick(string name_object,double distance_from_ee){
  ROS_INFO("Trying to attach the box with distance:%f",distance_from_ee);
  Pose pose;
  pose.position.x = 0.0;
  pose.position.y = distance_from_ee+0.0819;
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


//Stampa
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
void stampa_homo(Affine3d homo){
  cout<<"Translation: "<<endl<<homo.translation()<<endl;
  cout<<"Rotation: "<<endl<<homo.linear()<<endl;
}
void stampa_homo_named(Affine3d homo,string nome){
  cout<<"Homo:"<<nome<<endl;
  cout<<"Translation: "<<endl<<homo.translation()<<endl;
  cout<<"Rotation: "<<endl<<homo.linear()<<endl;
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
void stampa_aruco_su_excel(){

  ifstream inFile;
  string nome,tipo;
  string pkgpath = ros::package::getPath("ur3_control");
  string path_total_csv=pkgpath +"/../../../"+"aruco_table.csv";
  cout<<"percorso txt: "<<path_total_csv<<endl<<endl;



  std::ofstream myfile;
  myfile.open (path_total_csv);
  for(int i=0;i<aruco_length_array;i++){
    if(!Aruco_values[i].valid)
    {
      myfile <<i<<" "<<"0.000"<<" "<<"0.000"<<" "<<"0.000"<<"\n";
    }
    else{
      myfile <<i<<" "<<trunc((Aruco_values[i].pose.position.x)*1000)/1000<<" "<<trunc((Aruco_values[i].pose.position.y)*1000)/1000<<" "<<trunc((Aruco_values[i].pose.position.z)*1000)/1000<<"\n";
    }
  }

  myfile.close();
}

//Conversioni
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
void SetPoseOrientationRPY(Pose *p,int x0,int y0,int z0)
{
  tf2::Quaternion quat;
  quat.setRPY(grad_to_rad(x0),grad_to_rad(y0),grad_to_rad(z0));
  p->orientation.x=quat.getX();
  p->orientation.y=quat.getY();
  p->orientation.z=quat.getZ();
  p->orientation.w=quat.getW();
}
Pose homo_to_pose(Affine3d homo){

  tf::Pose pose_tf;
  Pose pose;
  tf::poseEigenToTF(homo,pose_tf);
  tf::poseTFToMsg(pose_tf,pose);
  return pose;

}
Affine3d pose_to_homo(Pose pose){
  Affine3d homo;
  tf::Pose pose_tf;
  tf::poseMsgToTF(pose,pose_tf);
  tf::poseTFToEigen(pose_tf,homo);
  return homo;

}
Matrix3d from_rpy_to_rotational_matrix(double roll,double pitch,double yaw){
  Quaterniond q;
  q = AngleAxisd(roll, Vector3d::UnitX())
      * AngleAxisd(pitch, Vector3d::UnitY())
      * AngleAxisd(yaw, Vector3d::UnitZ());
  return q.toRotationMatrix();

}


//Gripper
bool update_gripper_collision_box(string new_position){
  /*Diversamente dalle altre funzioni, qui verrà fatto tutto rispetto al tool. Questo per mantenere solidale la box*/
  Affine3d T_tool_to_center_of_box;




  PoseStamped box_pose;
  float box_size[3];
  string box_name="gripper_box";
  box_pose.header.frame_id="ee_link";
  box_size[1]=0.03;
  if(new_position=="open"){

    T_tool_to_center_of_box.translation().x()=(0.096+0.025/2)/2;
    T_tool_to_center_of_box.translation().y()=0.002;
    T_tool_to_center_of_box.translation().z()=0;
    T_tool_to_center_of_box.linear()=from_rpy_to_rotational_matrix(0,0,0);//non mi interessa cambiare la rotazione






    box_size[0]=0.12;
    box_size[2]=0.13;

  }
  if(new_position=="semi_open"){

    T_tool_to_center_of_box.translation().x()=(0.128+0.025/2)/2;
    T_tool_to_center_of_box.translation().y()=0.002;
    T_tool_to_center_of_box.translation().z()=0;
    T_tool_to_center_of_box.linear()=from_rpy_to_rotational_matrix(0,0,0);//non mi interessa cambiare la rotazione



    box_size[0]=(0.128+0.025/2)*1.1;
    box_size[2]=0.1;

  }
  if(new_position=="semi_close"){

    T_tool_to_center_of_box.translation().x()=(0.132+0.025/2)/2;
    T_tool_to_center_of_box.translation().y()=0.002;
    T_tool_to_center_of_box.translation().z()=0;
    T_tool_to_center_of_box.linear()=from_rpy_to_rotational_matrix(0,0,0);//non mi interessa cambiare la rotazione




    box_size[0]=(0.132+0.025/2)*1.1;
    box_size[2]=0.09;

  }
  if(new_position=="close"){

    T_tool_to_center_of_box.translation().x()=(0.135+0.025/2)/2;
    T_tool_to_center_of_box.translation().y()=0.002;
    T_tool_to_center_of_box.translation().z()=0;
    T_tool_to_center_of_box.linear()=from_rpy_to_rotational_matrix(0,0,0);//non mi interessa cambiare la rotazione



    box_size[0]=(0.135+0.025/2)*1.1;
    box_size[2]=0.07;

  }

  Pose pose_center_of_box=homo_to_pose(T_tool_to_center_of_box);
  box_pose.pose=pose_center_of_box;

  collision_boxes[box_name].name=box_name;
  collision_boxes[box_name].pose=box_pose;
  collision_boxes[box_name].size[0]=box_size[0];
  collision_boxes[box_name].size[1]=box_size[1];
  collision_boxes[box_name].size[2]=box_size[2];
  remove_and_detach_box(collision_boxes["gripper_box"]);
  add_and_attach_box(collision_boxes["gripper_box"]);
}
bool action_gripper(string input){


  moveit_msgs::MoveGroupActionResult msg;
  moveit_msgs::MoveGroupActionResultConstPtr msg_pointer;
  std_msgs::String msg_to_pub;
  ros::Duration d;
  ROS_INFO("TRYING TO SET GRIPPER AS:");
  cout<<input<<endl;

  d.sec=6;


  if(input==posizione_gripper){
    ROS_INFO("Gripper already in the correct position(STATE MACHINE)");
  }


  msg_to_pub.data=input;
  pub_gripper.publish(msg_to_pub);
  msg_pointer=(ros::topic::waitForMessage<moveit_msgs::MoveGroupActionResult>("/move_group/result",d));
  if(msg_pointer==NULL){

    ROS_INFO("NO MESSAGES RECEIVED");

    actionlib_msgs::GoalID msg_traj_cancel;
    msg_traj_cancel.id="";
    pub_traj_cancel.publish(msg_traj_cancel);
    return false;

  }
  msg=*msg_pointer;

  if(msg.status.text=="Requested path and goal constraints are already met.")
  {
    ROS_INFO("Gripper already in the correct position");
    return true;
  }
  if(msg.status.text=="Solution was found and executed."){

    ROS_INFO("Gripper solution found and executed");
    posizione_gripper=input;
    update_gripper_collision_box(input);
    return true;

  }if(msg.status.text=="No motion plan found. No execution attempted."){

    ROS_INFO("No solution found for the gripper movement");
    return false;

  }
  //ROS_INFO("size: %d",msg.result.planned_trajectory.joint_trajectory.joint_names.size());
  if(msg.result.planned_trajectory.joint_trajectory.joint_names.size()>0){
    if(msg.result.planned_trajectory.joint_trajectory.joint_names[0]=="finger_joint"){

      double final_value_gripper=-20;
      final_value_gripper=msg.result.planned_trajectory.joint_trajectory.points[msg.result.planned_trajectory.joint_trajectory.points.size()-1].positions[0];

      if(final_value_gripper>-0.2 && final_value_gripper<0.2){
        if(input=="open"){

          ROS_INFO("Gripper in the correct position");
          posizione_gripper=input;

          update_gripper_collision_box(input);
          return true;

        }
        else{

          ROS_INFO("Gripper ERROR??? Final value of gripper:%f",final_value_gripper);
          return false;

        }
      }
      if(final_value_gripper>=0.2 && final_value_gripper<0.85){
        if(input=="semi_open"){

          ROS_INFO("Gripper in the correct position");
          posizione_gripper=input;

          update_gripper_collision_box(input);
          return true;

        }
        else{

          ROS_INFO("Gripper ERROR??? Final value of gripper:%f",final_value_gripper);
          return false;

        }
      }
      if(final_value_gripper>=0.85 && final_value_gripper<1.18){
        if(input=="semi_close"){

          ROS_INFO("Gripper in the correct position");
          posizione_gripper=input;
          update_gripper_collision_box(input);
          return true;

        }
        else{

          ROS_INFO("Gripper ERROR??? Final value of gripper:%f",final_value_gripper);
          return false;

        }
      }
      if(final_value_gripper>=1.18){
        if(input=="close"){

          ROS_INFO("Gripper in the correct position");
          posizione_gripper=input;
          update_gripper_collision_box(input);
          return true;

        }
        else{

          ROS_INFO("Gripper ERROR??? Final value of gripper:%f",final_value_gripper);
          return false;

        }
      }

      ROS_INFO("finger joint letto, final value: %f",final_value_gripper);
    }

    ROS_INFO("finger joint non letto");
  }

  ROS_INFO("RESULT MESSAGE NON ANALIZZATO CORRETTAMENTE");

  return false;

}
bool calibrazione_gripper(){
  ROS_INFO("Codice della calibrazione non completato");
  return false;
}


//Movimenti
bool move_to_pose_cartesian(geometry_msgs::Pose pose){
  std::vector<Pose> waypoints;
  waypoints.push_back(pose);  // up and left
  RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double traj_threshold=1;

  double traj_erro=robot->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true);
  if(traj_erro>=traj_threshold){
    robot->execute(trajectory);
    return true;
  }
  else{
    //ROS_INFO("Traj error:%f",traj_erro);
    traj_erro=robot->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory,true);
    if(traj_erro>=traj_threshold){
      robot->execute(trajectory);
      return true;
    }
    else{
      //ROS_INFO("Traj error:%f",traj_erro);

      traj_erro=robot->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory,true);
      if(traj_erro>=traj_threshold){
        robot->execute(trajectory);
        return true;
      }
      else{
        ROS_INFO("Traj error:%f",traj_erro);

        return false;
      }
    }
  }

}
bool move_to_pose(geometry_msgs::Pose pt, bool Orientamento){

  if(!Orientamento)
    robot->setPositionTarget(pt.position.x,pt.position.y,pt.position.z);
  else
    robot->setPoseTarget(pt);

  for(int i=0;i<4;i++){
    success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
    if(success){
      i=5;
      //ROS_INFO_NAMED("tutorial", "Risultato:%s", success ? "SUCCESS" : "FAILED");
      robot->execute(my_plan);
      return true;
    }
    else{
      ROS_INFO("%d pianificazione fallita",i);
    }
  }
  return false;
}
bool move_to_joints(vector<double> joint_group_positions){
  robot->setJointValueTarget(joint_group_positions);
  success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
  if(success){
    robot->move();
    return true;
  }
  else{
    ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
    return false;
  }
}
bool move_to_pose_optimized(geometry_msgs::Pose pose){
  robot->setPlanningTime(7);
  if(move_to_pose_cartesian(pose)){
      return true;
  }
  if(move_to_pose(pose,true)){
    return true;
  }
  stampa_Pose(pose);

  return false;
}
bool ritorno_al_pannello(double percentual){
  ROS_INFO("ERC:Starting going back to pannello");
  Pose actual_pose,pann_pose,final_pose;

  if(!pose_pannello_elaborata.valid)
    return false;


  actual_pose=robot->getCurrentPose().pose;
  pann_pose=pose_pannello_elaborata.pose;

  final_pose=actual_pose;

  final_pose.position.x= final_pose.position.x + (pann_pose.position.x-actual_pose.position.x)*percentual/100;
  //final_pose.position.y= final_pose.position.y + (pann_pose.position.y-actual_pose.position.y)*percentual/100;
  //final_pose.position.z= final_pose.position.z + (pann_pose.position.z-actual_pose.position.z)*percentual/100;

  return move_to_pose_cartesian(final_pose);
}
Pose pre_grasp(Affine3d T_0_aruco){
  Affine3d T_aruco_finalpos,T_0_finalpos;
  Matrix3d rotation_aruco_final_pos;
  Vector3d xaf(0,0,1),yaf(0,1,0),zaf(-1,0,0),trans_aruco_finalpos(0,0,0.22);//con il gripper che punta sull'aruco


  rotation_aruco_final_pos.row(0)=xaf;
  rotation_aruco_final_pos.row(1)=yaf;
  rotation_aruco_final_pos.row(2)=zaf;

  T_aruco_finalpos.linear()=rotation_aruco_final_pos;
  T_aruco_finalpos.translation()=trans_aruco_finalpos;

  T_0_finalpos=T_0_aruco*T_aruco_finalpos;
  //In questo punto ho T_0_final

  move_to_pose(homo_to_pose(T_0_finalpos),true);
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
    if(pose_pannello_elaborata.valid)
      robot->setPoseTarget(pose_pannello_elaborata.pose);
    else {
      ROS_INFO("Pannello non ancora individuato");
      return false;
    }
  }
  else
  if(str_posizione==str_rotaz_pannello){
    vector<double> pos_joint_rotaz_pannello=pos_joint_pannello;
    pos_joint_rotaz_pannello[0]=robot->getCurrentJointValues().at(0);//il primo giunto non viene fissato, rimane uguale a quello corrente
    robot->setJointValueTarget(pos_joint_rotaz_pannello);

  }
  else
  if(str_posizione==str_pos_iniziale){
    joint_group_positions=pos_joint_iniziale;
    robot->setJointValueTarget(joint_group_positions);
    }
  else
  if(str_posizione==str_pos_iniziale_cam_alta){
    joint_group_positions=pos_joint_iniziale_cam_alta;
    robot->setJointValueTarget(joint_group_positions);
  }
  else
  if(str_posizione==str_pos_adatta_ricerca_per_terra){
    joint_group_positions=pos_joint_adatta_ricerca_per_terra;
    robot->setJointValueTarget(joint_group_positions);
  }
  else{
    ROS_INFO("Posizione non presente tra quelle base");
    return false;
  }

  success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
  if(success){
    robot->move();
    return true;
  }
  else{
    ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
    return false;
  }

}
void ruota_360(){
  ROS_INFO("Starting rotating 360");



  vector<double> joint_positions;
  joint_positions=robot->getCurrentJointValues();
  joint_positions[0] = joint_positions[0]-grad_to_rad(360);  // radians
  robot->setJointValueTarget(joint_positions);
  success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);

  if(!success){

    joint_positions=robot->getCurrentJointValues();
    joint_positions[0] = joint_positions[0]+grad_to_rad(-270);  // radians
    robot->setJointValueTarget(joint_positions);
    success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);

    if(!success){

      joint_positions=robot->getCurrentJointValues();
      joint_positions[0] = joint_positions[0]+grad_to_rad(-180);  // radians
      robot->setJointValueTarget(joint_positions);
      success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);

      if(!success){

        joint_positions=robot->getCurrentJointValues();
        joint_positions[0] = joint_positions[0]+grad_to_rad(-90);  // radians
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

  if(!aruco_individuato()){

    bridge_service(str_md_bpa,"");

    ruota_360();
    if(!aruco_individuato()){
      ROS_INFO("GIRO COMPLETATO SENZA ARUCO");

    }
  }




}
void ruota_e_cerca_pannello(){
  //questa funzione setta come aruco da cercare il numero:1


  bridge_service(str_md_next_aruco,"1");

  usleep(100000);

  if(!aruco_individuato()){

    PosizioniBase(str_rotaz_pannello);


    if(!aruco_individuato()){
      bridge_service(str_md_bpa,"");

      ruota_360();
      if(!aruco_individuato()){
        ROS_INFO("GIRO COMPLETATO SENZA AVER TROVATO IL PANNELLO");
      }
}
}

  move_aruco_to_center_of_camera(0);
  move_aruco_to_center_of_camera(-20);
  move_aruco_to_center_of_camera(-20);

  if(aruco_individuato()){
    pose_pannello_elaborata.valid=true;
    pose_pannello_elaborata.pose=robot->getCurrentPose().pose;
    ROS_INFO("Posizione pannello aggiornata");
  }


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
bool allineamento_w_z_x_y(Pose target){
//Questa funzione raggiunge il target allineando prima l'orientamento poi z poi x poi y
  Pose target_w,target_x,target_y,target_z;
  target_w=robot->getCurrentPose().pose;
  target_w.orientation=target.orientation;

  if(!move_to_pose_optimized(target_w)){
    return false;
  }

  target_z=robot->getCurrentPose().pose;
  target_z.position.z=target.position.z;

  if(!move_to_pose_optimized(target_z)){
    return false;
  }


  target_x=robot->getCurrentPose().pose;
  target_x.position.x=target.position.x;

  if(!move_to_pose_optimized(target_x)){
    return false;
  }


  if(!move_to_pose_optimized(target)){
    return false;
  }

return true;
}
bool allineamento_x_y_w_z(Pose target){
//Questa funzione raggiunge il target allineando prima l'orientamento poi z poi x poi y
  Pose target_w,target_x,target_y,target_z;



  target_x=robot->getCurrentPose().pose;
  target_x.position.x=target.position.x;
  if(!move_to_pose_optimized(target_x)){
    return false;
  }

  target_y=robot->getCurrentPose().pose;
  target_y.position.y=target.position.y;
  if(!move_to_pose_optimized(target_y)){
    return false;
  }

  target_w=robot->getCurrentPose().pose;
  target_w.orientation=target.orientation;
  if(!move_to_pose_optimized(target_w)){
    return false;
  }




  if(!move_to_pose_optimized(target)){
    return false;
  }

return true;
}


//Trasformate omogenee
Affine_valid homo_0_aruco_elaration(){
  //elabora la trasformata omogenea da 0 ad aruco


  ur3_control::aruco_serviceResponse msg_from_bridge=bridge_service(str_md_rd,"");

  if(msg_from_bridge.aruco_found){

    Pose pose_robot=robot->getCurrentPose().pose;

    Pose pose_robot_target,pose_finalpos,pose_aruco,pose_camera;
    Affine3d T_0_tool,T_0_camera,T_camera_aruco,T_0_aruco,T_0_camera_gazebo;
    tf::Pose pose_robot_tf,pose_target_tf,pose_finalpos_tf,pose_aruco_tf,pose_camera_tf;
    tf::poseMsgToTF(pose_robot,pose_robot_tf);
    tf::poseTFToEigen(pose_robot_tf,T_0_tool);
    //In questo punto ho T_0_tool

    T_0_camera=T_0_tool*T_tool_camera;
    pose_camera=homo_to_pose(T_0_camera);
    //In questo punto ho T_0_camera

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

    bool debug=false;
    if(debug){
      cout<<"ee pose:"<<endl;
      stampa_Pose(pose_robot);

      cout<<"camera pose:"<<endl;
      stampa_Pose(pose_camera);

      cout<<"aruco pose:"<<endl;
      stampa_Pose(pose_aruco);

      cout<<"final pose:"<<endl;

      stampa_Pose(pose_finalpos);
    }


    Affine_valid return_value;
    return_value.homo_matrix=T_0_aruco;
    return_value.valid=true;
    return return_value;

  }else{
    ROS_INFO("Hai chiamato il calcolo della trasformata T_0_aruco ma non vedo nessun aruco");
    Affine_valid return_value;
    return_value.valid=false;
    return return_value;

  }


}
bool move_aruco_to_center_of_camera(double percentual_zoom){
  ur3_control::aruco_serviceResponse msg_from_bridge=bridge_service(str_md_rd,"");

  if(msg_from_bridge.aruco_found){
    ROS_INFO("ERC: centering camera");
    Pose pose_robot=robot->getCurrentPose().pose;

    Pose pose_robot_target,pose_finalpos,pose_aruco,pose_camera;
    Affine3d T_0_tool,T_0_camera,T_camera_aruco,T_camera_aruco_modified,T_0_aruco,T_0_camera_gazebo;
    tf::Pose pose_robot_tf,pose_target_tf,pose_finalpos_tf,pose_aruco_tf,pose_camera_tf;

    tf::poseMsgToTF(pose_robot,pose_robot_tf);
    tf::poseTFToEigen(pose_robot_tf,T_0_tool);
    //In questo punto ho T_0_tool

    T_0_camera=T_0_tool*T_tool_camera;
    tf::poseEigenToTF(T_0_camera,pose_camera_tf);
    tf::poseTFToMsg(pose_camera_tf,pose_camera);
    //In questo punto ho T_0_camera


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



    Affine3d T_0_tool_modified,T_tool_ee,T_0_ee_modified,T_0_camera_modified,T_aruco_camera_modified,T_my_rotation,T_my_translation;
    Pose pose_ee_modified,pose_camera_modified;
    tf::Pose pose_ee_modified_tf,pose_cam_mod_tf;



    //calcolo vettore che collega 0_camera_gazebo con aruco
    Vector3d vettore_gazebo_aruco(0,0,0);
    vettore_gazebo_aruco.x()=T_0_aruco.translation().x()-T_0_camera_gazebo.translation().x();
    vettore_gazebo_aruco.y()=T_0_aruco.translation().y()-T_0_camera_gazebo.translation().y();
    vettore_gazebo_aruco.z()=T_0_aruco.translation().z()-T_0_camera_gazebo.translation().z();
    Matrix3d rotation_my_rot;
    Vector3d x_my_rot(1,0,0),y_my_rot(0,1,0),z_my_rot(0,0,0),trans_my_rot(0,0,0);
    rotation_my_rot.col(0)=x_my_rot;
    rotation_my_rot.col(1)=y_my_rot;
    rotation_my_rot.col(2)=vettore_gazebo_aruco;
    T_my_rotation.translation()=trans_my_rot;
    T_my_rotation.linear()=rotation_my_rot;

    Affine3d T_0_camera_gazebo_modified,T_all_orizz,T_all_vert,T_0_camera_gazebo_modified_orizz,T_camera_aruco_modified_orizz,T_all_rotativo;


    //float roll = 0, pitch = 0, yaw = atan(T_camera_aruco.translation().y()/T_camera_aruco.translation().x());
    float roll = 0, pitch = 0, yaw = atan(vettore_gazebo_aruco.y()/vettore_gazebo_aruco.x());
    //
    //

    Quaterniond q;
    q = AngleAxisd(roll, Vector3d::UnitX())
        * AngleAxisd(pitch, Vector3d::UnitY())
        * AngleAxisd(yaw, Vector3d::UnitZ());
    Matrix3d rot_allineamento_orizzontale=q.toRotationMatrix();

    T_all_orizz.translation().x()=0;
    T_all_orizz.translation().y()=0;
    T_all_orizz.translation().z()=0;
    T_all_orizz.linear()=rot_allineamento_orizzontale;


    //Applicando la matrice sopra a T_0_camera_gazebo, ottengo che l'asse blu è esattamente sopra all'aruco. se rotazione su asse verde allora perfetto
    // mi serve la nuova distanza tra la camera e l'aruco sull'asse rosso

    T_0_camera_gazebo_modified_orizz=T_0_camera_gazebo;
    T_0_camera_gazebo_modified_orizz.linear()=rot_allineamento_orizzontale;
    T_camera_aruco_modified_orizz=T_0_camera_gazebo_modified_orizz.inverse()*T_0_aruco;

    double dissallineamento_verticale=M_PI/2 - atan(T_camera_aruco_modified_orizz.translation().z()/T_camera_aruco_modified_orizz.translation().x());
    double sq_dist=sqrt(vettore_gazebo_aruco.x()*vettore_gazebo_aruco.x() +vettore_gazebo_aruco.y()*vettore_gazebo_aruco.y() + vettore_gazebo_aruco.z()*vettore_gazebo_aruco.z());


    roll = 0, pitch = dissallineamento_verticale, yaw = 0;
    Quaterniond q_all_vert;
    q_all_vert = AngleAxisd(roll, Vector3d::UnitX())
        * AngleAxisd(pitch, Vector3d::UnitY())
        * AngleAxisd(yaw, Vector3d::UnitZ());
    Matrix3d rot_all_vert=q_all_vert.toRotationMatrix();

    T_all_vert.translation().x()=0;
    T_all_vert.translation().y()=0;
    T_all_vert.translation().z()=0;
    T_all_vert.linear()=rot_all_vert;


    roll = 0, pitch = 0, yaw = -M_PI/2;
    Quaterniond q_all_rotativo;
    q_all_rotativo = AngleAxisd(roll, Vector3d::UnitX())
        * AngleAxisd(pitch, Vector3d::UnitY())
        * AngleAxisd(yaw, Vector3d::UnitZ());
    Matrix3d rot_all_rotativo=q_all_rotativo.toRotationMatrix();

    T_all_rotativo.translation().x()=0;
    T_all_rotativo.translation().y()=0;
    T_all_rotativo.translation().z()=0;
    T_all_rotativo.linear()=rot_all_rotativo;




    T_0_camera_gazebo_modified=T_0_camera_gazebo;
    T_0_camera_gazebo_modified.linear()=rot_allineamento_orizzontale;
    T_0_camera_gazebo_modified=T_0_camera_gazebo_modified*T_all_vert*T_all_rotativo;

    //ZOOM
    //T_0_camera*T_camera_aruco_mod=T_0_aruco
    T_camera_aruco_modified=T_0_camera_gazebo_modified.inverse()*T_0_aruco;
    double distance_camera_aruco_z=T_camera_aruco_modified.translation().z();

    Affine3d T_zoom;
    T_zoom.linear().setIdentity();
    T_zoom.translation().z()=percentual_zoom*distance_camera_aruco_z/100;
    T_0_camera_gazebo_modified=T_0_camera_gazebo_modified*T_zoom;




    T_0_camera_modified=T_0_camera_gazebo_modified*T_camera_camera_gazebo.inverse();
    tf::poseEigenToTF(T_0_camera_modified,pose_cam_mod_tf);
    tf::poseTFToMsg(pose_cam_mod_tf,pose_camera_modified);


    T_0_tool_modified=T_0_camera_modified*T_tool_camera.inverse();


    pose_ee_modified=homo_to_pose(T_0_tool_modified);
    bool debug=false;
    if(debug){
      cout<<"Translation camera aruco modified orizz:"<<endl<<T_camera_aruco_modified_orizz.translation();

      cout<<"Rotation with rpy:"<<endl<<T_0_camera_gazebo_modified.linear();
      cout<<"Translation with rpy:"<<endl<<T_0_camera_gazebo_modified.translation();


      cout<<"Disallineamento verticale:"<<dissallineamento_verticale<<endl;
      cout<<"ee pose:"<<endl;
      stampa_Pose(pose_robot);

      cout<<"camera pose:"<<endl;
      stampa_Pose(pose_camera);

      cout<<"camera_modified pose:"<<endl;
      stampa_Pose(pose_camera_modified);

      cout<<"pose ee_modified:"<<endl;

      stampa_Pose(pose_ee_modified);

      cout<<"MY_TRANSLATION translation:"<<endl<<T_my_translation.translation()<<endl;

      cout<<"MY_TRANSLATION rotation:"<<endl<<T_my_translation.linear()<<endl;

      cout<<"MY_ROTATION translation:"<<endl<<T_my_rotation.translation()<<endl;

      cout<<"MY_ROTATION rotation:"<<endl<<T_my_rotation.linear()<<endl;

      cout<<"T_0_camera translation:"<<endl<<T_0_camera.translation()<<endl;

      cout<<"T_0_camera rotation:"<<endl<<T_0_camera.linear()<<endl;

      cout<<"T_0_camera_modified translation:"<<endl<<T_0_camera_modified.translation()<<endl;

      cout<<"T_0_camera_modified rotation:"<<endl<<T_0_camera_modified.linear()<<endl;


      cout<<"T_camera_aruco translation:"<<endl<<T_camera_aruco.translation()<<endl;

      cout<<"T_camera_aruco rotation:"<<endl<<T_camera_aruco.linear()<<endl;


      cout<<"T_camera_aruco modified translation:"<<endl<<T_camera_aruco_modified.translation()<<endl;

      cout<<"T_camera_aruco modified rotation:"<<endl<<T_camera_aruco_modified.linear()<<endl;

      cout<<"T_camera_aruco modified MANUAL INVERSE translation:"<<endl<<T_aruco_camera_modified.translation()<<endl;

      cout<<"T_camera_aruco modified MANUAL INVERSE rotation:"<<endl<<T_aruco_camera_modified.linear()<<endl;

      cout<<"T_camera_aruco modified INVERSE translation:"<<endl<<T_camera_aruco_modified.inverse().translation()<<endl;

      cout<<"T_camera_aruco modified INVERSE rotation:"<<endl<<T_camera_aruco_modified.inverse().linear()<<endl;

    }


//      move_to_pose(pose_ee_modified,true);
//      return true;
    return move_to_pose_cartesian(pose_ee_modified);

  }
  else{
    ROS_INFO("No aruco detected, no return??");
    return false;
  }

}
bool centra_aruco_nella_camera(int ID_aruco, double percentuale_zoom){

  if(!Aruco_values[ID_aruco].valid){
    ROS_INFO("NON POSSEGGO QUELL ARUCO");
    return false;
  }

  Affine3d T_0_aruco,T_0_tool,T_0_camera_gazebo,T_camera_gazebo_aruco,T_0_camera_gazebo_modified,T_all_orizz,T_all_vert,T_0_camera_gazebo_modified_orizz,T_camera_gazebo_aruco_modified_orizz,T_all_rotativo,T_0_tool_modified_oriz;
  Vector3d vettore_gazebo_aruco(0,0,0);
  Matrix3d rot_allineamento_orizzontale,rot_all_vert,rot_all_rotativo;
  float roll_oriz, pitch_oriz, yaw_oriz,roll_vert,pitch_vert,yaw_vert,roll_rot,pitch_rot,yaw_rot;
  T_0_tool=pose_to_homo(robot->getCurrentPose().pose);
  T_0_aruco=pose_to_homo(Aruco_values[ID_aruco].pose);
  T_0_camera_gazebo=T_0_tool*T_tool_camera_gazebo;

  //T_0_aruco=T_0_camera*T_camera_aruco

  T_camera_gazebo_aruco=T_0_camera_gazebo.inverse()*T_0_aruco;
  Affine3d T_tool_aruco;
  Affine3d T_ok;
  T_tool_aruco=T_0_tool.inverse()*T_0_aruco;

  stampa_homo_named(T_tool_aruco,"T_tool_aruco");
  float theta1,theta2;

  //con servizio python------------------
  //ros::NodeHandle node_handle;
  //ros::ServiceClient client1;
  //client1 = node_handle.serviceClient<ur3_control::float_return_srv>("/solve_equation_serv");
  //ur3_control::float_return_srv msg;
  //msg.request.num1=T_tool_aruco.translation().x();
  //msg.request.num2=T_tool_aruco.translation().y();
  //client1.call(msg);
  //--------------------------

  float Xa=T_tool_aruco.translation().x();
  float Ya=T_tool_aruco.translation().y();
  theta1=2*atan((5000*Xa*cos(pi/18) + 5000*Ya*sin(pi/18) + sqrt((25000000*pow(Xa,2)*pow(cos(pi/18),2) - 190969*pow(cos(pi/18),2) + 25000000*pow(Ya,2)*pow(cos(pi/18),2) + 25000000*pow(Xa,2)*pow(sin(pi/18),2) + 25000000*pow(Ya,2)*pow(sin(pi/18),2))))/(437*cos(pi/18) - 5000*Ya*cos(pi/18) + 5000*Xa*sin(pi/18)));
  theta2=2*atan((5000*Xa*cos(pi/18) + 5000*Ya*sin(pi/18) - sqrt((25000000*pow(Xa,2)*pow(cos(pi/18),2) - 190969*pow(cos(pi/18),2) + 25000000*pow(Ya,2)*pow(cos(pi/18),2) + 25000000*pow(Xa,2)*pow(sin(pi/18),2) + 25000000*pow(Ya,2)*pow(sin(pi/18),2))))/(437*cos(pi/18) - 5000*Ya*cos(pi/18) + 5000*Xa*sin(pi/18)));
  printf("res1: %f \n",theta1);
  printf("res2: %f \n",theta2);

  Affine3d T_finale;

{
  Matrix3d rot_ok;
  rot_ok=from_rpy_to_rotational_matrix(0,0,theta1);
  T_ok.linear()=rot_ok;

  T_ok.translation().x()=0;
  T_ok.translation().y()=0;
  T_ok.translation().z()=0;

  Affine3d T_0_tool_vert,T,T_tool_aruco_vert;
  T_0_tool_vert=T_0_tool*T_ok;

  T_tool_aruco_vert=T_0_tool_vert.inverse()*T_0_aruco;


  //move_to_pose_optimized(homo_to_pose(T_0_tool_vert));
  //sleep(2);

  double phi=0;
  phi=-(M_PI/2-atan2(T_tool_aruco_vert.translation().x(),T_tool_aruco_vert.translation().z()));



  Affine3d T_orizz;
  Matrix3d rot_orizz;
  rot_orizz=from_rpy_to_rotational_matrix(0,phi,0);
  T_orizz.linear()=rot_orizz;

  T_orizz.translation().x()=0;
  T_orizz.translation().y()=0;
  T_orizz.translation().z()=0;

  Affine3d T_0_tool_vert_orizz,T_tool_aruco_theta1;
  T_0_tool_vert_orizz=T_0_tool_vert*T_orizz;

  T_tool_aruco_theta1=T_0_tool_vert_orizz.inverse()*T_0_aruco;
  stampa_homo_named(T_tool_aruco_theta1,"T_tool_aruco_theta1");
  if(T_tool_aruco_theta1.translation().x()>0){
    T_finale=T_0_tool_vert_orizz;
  }
}
{

    Matrix3d rot_ok;
    rot_ok=from_rpy_to_rotational_matrix(0,0,theta2);
    T_ok.linear()=rot_ok;

    T_ok.translation().x()=0;
    T_ok.translation().y()=0;
    T_ok.translation().z()=0;

    Affine3d T_0_tool_vert,T,T_tool_aruco_vert;
    T_0_tool_vert=T_0_tool*T_ok;

    T_tool_aruco_vert=T_0_tool_vert.inverse()*T_0_aruco;


    //move_to_pose_optimized(homo_to_pose(T_0_tool_vert));
    //sleep(2);

    double phi=0;
    phi=-(M_PI/2-atan2(T_tool_aruco_vert.translation().x(),T_tool_aruco_vert.translation().z()));



    Affine3d T_orizz;
    Matrix3d rot_orizz;
    rot_orizz=from_rpy_to_rotational_matrix(0,phi,0);
    T_orizz.linear()=rot_orizz;

    T_orizz.translation().x()=0;
    T_orizz.translation().y()=0;
    T_orizz.translation().z()=0;

    Affine3d T_0_tool_vert_orizz,T_tool_aruco_theta2;
    T_0_tool_vert_orizz=T_0_tool_vert*T_orizz;

    T_tool_aruco_theta2=T_0_tool_vert_orizz.inverse()*T_0_aruco;
    stampa_homo_named(T_tool_aruco_theta2,"T_tool_aruco_theta1");
    if(T_tool_aruco_theta2.translation().x()>0){
      T_finale=T_0_tool_vert_orizz;
    }
}

  move_to_pose_optimized(homo_to_pose(T_finale));

//  stampa_homo_named(T_tool_camera_gazebo,"T_tool_camera_gazebo");
//  stampa_homo_named(T_0_aruco,"T_0_aruco");
//  stampa_homo_named(T_0_camera_gazebo,"T_0_camera_gazebo");
//  stampa_homo_named(T_0_tool,"T_0_tool");
//  stampa_homo_named(T_camera_gazebo_aruco,"T_camera_gazebo_aruco");
//  stampa_homo_named(T_ok,"T_ok");

  return true;
}
/*bool zoom_camera_to_aruco(int ID_aruco,double valore){
  if(!Aruco_values[ID_aruco].valid){
    ROS_INFO("NON POSSEGGO QUELL ARUCO");
    return false;
  }
  cambia_aruco(to_string(ID_aruco));
  if(!aruco_individuato()){
    centra_aruco_nella_camera(ID_aruco, 0);
  }


  Affine3d T_0_aruco,T_0_tool,T_0_camera_gazebo,T_camera_gazebo_aruco,T_0_camera_gazebo_modified,T_all_orizz,T_all_vert,T_0_camera_gazebo_modified_orizz,T_camera_gazebo_aruco_modified_orizz,T_all_rotativo,T_0_tool_modified_oriz,T_0_tool_zoom;
  Vector3d vettore_gazebo_aruco(0,0,0);
  Matrix3d rot_allineamento_orizzontale,rot_all_vert,rot_all_rotativo;
  float roll_oriz, pitch_oriz, yaw_oriz,roll_vert,pitch_vert,yaw_vert,roll_rot,pitch_rot,yaw_rot;
  T_0_tool=pose_to_homo(robot->getCurrentPose().pose);
  T_0_aruco=pose_to_homo(Aruco_values[ID_aruco].pose);
  T_0_camera_gazebo=T_0_tool*T_tool_camera_gazebo;

  //T_0_aruco=T_0_camera*T_camera_aruco

  T_camera_gazebo_aruco=T_0_camera_gazebo*T_0_aruco.inverse();
  float distanza_su_asse_focale=T_camera_gazebo_aruco.translation().z();

  Affine3d T_ok;
  T_ok.translation().z()=distanza_su_asse_focale*valore/100;
  T_ok.translation().x()=0;
  T_ok.translation().y()=0;
  T_ok.linear()=from_rpy_to_rotational_matrix(0,0,0);

  T_0_camera_gazebo=T_0_camera_gazebo*T_ok;

  T_0_tool_zoom=T_0_camera_gazebo*T_tool_camera_gazebo.inverse();

  move_to_pose_optimized(homo_to_pose(T_0_tool_zoom));

}
*/
bool function_pose_aruco(){
  ur3_control::aruco_serviceResponse msg_from_bridge=bridge_service(str_md_rd,"");

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
    bool debug=false;
    if(debug){
      cout<<"ee pose:"<<endl;
      stampa_Pose(pose_robot);

      cout<<"camera pose:"<<endl;
      stampa_Pose(pose_camera);

      cout<<"aruco pose:"<<endl;
      stampa_Pose(pose_aruco);

      cout<<"final pose:"<<endl;

      stampa_Pose(pose_finalpos);
    }
    move_to_pose(pose_finalpos,true);
    robot->setPlanningTime(std_planning_time);
    return true;

  }
  else{
    ROS_INFO("No aruco detected, no return??");
    return false;
  }

}

//Esplorazione
bool esplora_inspection_cover_storage(){



  cambia_aruco(to_string(ID_INSPECTION_WINDOW_COVER_STORAGE));


  if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER_STORAGE)) {
    return true;
  }

  bridge_service(str_md_bpa,"");

  PosizioniBase(str_pos_iniziale);

  if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER_STORAGE)) {
    return true;
  }


  vector<double> joint_group_positions=robot->getCurrentJointValues();
  joint_group_positions=pos_joint_iniziale;
  joint_group_positions[3]=grad_to_rad(77);
  joint_group_positions[5]=grad_to_rad(-90);
  robot->setJointValueTarget(joint_group_positions);
  success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
  robot->move();

  if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER_STORAGE)) {
    return true;
  }



  ruota_360();


  if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER_STORAGE)) {
    return true;
  }


  return false;



}
bool esplora_inspection_window_cover(){


  ROS_INFO("INIZIO ESPLORAZIONE, POTRESTI LEGGERE QUALCHE ABORTED");
  cambia_aruco(to_string(ID_INSPECTION_WINDOW_COVER));

  //TESTO ESPLORAZIONE ELIMINARE RIGA SOTTO

  //cambia_aruco(to_string(2));


  if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER))
    return true;

  bridge_service(str_md_bpa,"");

  //ROUTINE NUMBER 0:
  {


    PosizioniBase(str_pos_iniziale_cam_alta);

    Pose pose_final_pose_panel,pose_final_pose_ground;


    //RUOTO posizionandomi forse davanti all aruco




    vector<double> joint_group_positions=robot->getCurrentJointValues();
    joint_group_positions[0] = joint_group_positions[0]- grad_to_rad(35);  // radians
    robot->setJointValueTarget(joint_group_positions);
    success = (robot->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
    robot->move();
    //mi trovo a 18 gradi

    if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER))
      return true;
    if(!aruco_individuato()){

      bridge_service(str_md_bpa,"");

      //ruoto la testa cercandolo

      joint_group_positions[3]=joint_group_positions[3]+grad_to_rad(45);
      robot->setJointValueTarget(joint_group_positions);
      success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
      robot->move();


      if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER))
        return true;

    }

    ROS_INFO("The first routine to find inspection window has failed, trying with the next one");
  }

  //RUOTINE NUMBERO 1:
  {

    PosizioniBase(str_pos_iniziale);
    if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER))
      return true;

    vector<double> joint_group_positions=pos_joint_iniziale;

    joint_group_positions=pos_joint_iniziale;
    joint_group_positions[0]=grad_to_rad(0);
    joint_group_positions[1]=grad_to_rad(-110);
    joint_group_positions[2]=grad_to_rad(90);
    joint_group_positions[3]=grad_to_rad(-110);
    joint_group_positions[4]=grad_to_rad(-90);
    joint_group_positions[5]=grad_to_rad(0);
    robot->setJointValueTarget(joint_group_positions);
    success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
    robot->move();


    if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER))
      return true;

    joint_group_positions[0]=grad_to_rad(-100);
    robot->setJointValueTarget(joint_group_positions);
    success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
    robot->move();

    if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER))
      return true;


    joint_group_positions[3]=grad_to_rad(-90);
    robot->setJointValueTarget(joint_group_positions);
    success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
    robot->move();

    if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER))
      return true;

    joint_group_positions[0]=grad_to_rad(0);
    robot->setJointValueTarget(joint_group_positions);
    success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
    robot->move();

    if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER))
      return true;


  }

  //ROUTINE NUMBER 2:
  {

    PosizioniBase(str_pos_iniziale_cam_alta);

    if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER))
      return true;

    //Posizione pre rotazione


    vector<double> joint_group_positions=pos_joint_iniziale;
    joint_group_positions[3] = grad_to_rad(50);  // radians
    joint_group_positions[5] = grad_to_rad(-90);  // radians
    robot->setJointValueTarget(joint_group_positions);
    success = (robot->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
    robot->move();

    joint_group_positions[0]=joint_group_positions[0]-grad_to_rad(100);
    robot->setJointValueTarget(joint_group_positions);
    success = (robot->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
    robot->move();

    if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER))
      return true;

  }


  if(aruco_individuato()) {
    sleep(2);
    Affine_valid T_0_aruco_valid=homo_0_aruco_elaration();

    if(! T_0_aruco_valid.valid){
      //esplorazione pannello centrale
      return false;
    }

    Aruco_values[ID_INSPECTION_WINDOW_COVER].valid=true;
    Aruco_values[ID_INSPECTION_WINDOW_COVER].pose=homo_to_pose(T_0_aruco_valid.homo_matrix);

    return true;
  }
  else return false;
}
bool esplora_cerca_IMU(){
  cambia_aruco(to_string(ID_IMU_MODULE));
  sleep(1);


  PosizioniBase(str_pos_iniziale_cam_alta);


  vector<double> joint_group_positions=robot->getCurrentJointValues();
  joint_group_positions=pos_joint_iniziale_cam_alta;
  joint_group_positions[3]=grad_to_rad(77);
  joint_group_positions[5]=grad_to_rad(-90);
  move_to_joints(joint_group_positions);

  if(se_aruco_individuato_aggiorna_array(ID_IMU_MODULE)) {
    return true;
  }

  bridge_service(str_md_bpa,"");


  joint_group_positions=robot->getCurrentJointValues();
  joint_group_positions[0]=joint_group_positions[0]+180;
  move_to_joints(joint_group_positions);

  if(se_aruco_individuato_aggiorna_array(ID_IMU_MODULE)) {
    return true;
  }

  return false;



}
bool esplora_cerca_left_panel_IMU(){
  ROS_INFO("Inizio esplorazione per trovare aruco IMU DESTINATION");
  cambia_aruco(to_string(ID_IMU_DESTINATION_PLANE));
  if(se_aruco_individuato_aggiorna_array(ID_IMU_DESTINATION_PLANE))
    return true;

  bridge_service(str_md_bpa,"");

  PosizioniBase(str_pos_iniziale);

  if(se_aruco_individuato_aggiorna_array(ID_IMU_DESTINATION_PLANE))
    return true;
//ROUTINE 1
{
  vector<double> joint_group_positions=robot->getCurrentJointValues();
  joint_group_positions=pos_joint_iniziale;
  joint_group_positions[0]=grad_to_rad(0);
  joint_group_positions[1]=grad_to_rad(-136.929);
  joint_group_positions[2]=grad_to_rad(103.488);
  joint_group_positions[3]=grad_to_rad(20);
  joint_group_positions[4]=grad_to_rad(90);
  joint_group_positions[5]=grad_to_rad(90);

  if(!move_to_joints(joint_group_positions)){
    return false;
  }

  if(se_aruco_individuato_aggiorna_array(ID_IMU_DESTINATION_PLANE))
    return true;


  joint_group_positions[0]=grad_to_rad(90);
  if(!move_to_joints(joint_group_positions)){
    return false;
  }

  if(se_aruco_individuato_aggiorna_array(ID_IMU_DESTINATION_PLANE))
    return true;


  joint_group_positions[3]=grad_to_rad(40);
  if(!move_to_joints(joint_group_positions)){
    return false;
  }

  joint_group_positions[0]=grad_to_rad(0);
  if(!move_to_joints(joint_group_positions)){
    return false;
  }

  if(se_aruco_individuato_aggiorna_array(ID_IMU_DESTINATION_PLANE))
    return true;

  joint_group_positions[3]=grad_to_rad(60);
  if(!move_to_joints(joint_group_positions)){
    return false;
  }

  if(se_aruco_individuato_aggiorna_array(ID_IMU_DESTINATION_PLANE))
    return true;

  joint_group_positions[0]=grad_to_rad(90);
  if(!move_to_joints(joint_group_positions)){
    return false;
  }

  if(se_aruco_individuato_aggiorna_array(ID_IMU_DESTINATION_PLANE))
    return true;
}

  return false;
}
bool esplorazione_middle_panel_per_trovare_aruco(string ID_str){
  stringstream ss;
  int ID_int;

  ss<<ID_str;
  ss >> ID_int;

  ROS_INFO("Inizio esplorazione per trovare aruco id:%d",ID_int);
  cambia_aruco(ID_str);
  if(se_aruco_individuato_aggiorna_array(ID_int))
    return true;

  bridge_service(str_md_bpa,"");

  PosizioniBase(str_pos_iniziale);

  if(se_aruco_individuato_aggiorna_array(ID_int))
    return true;
//ROUTINE 1
{
  vector<double> joint_group_positions=robot->getCurrentJointValues();
  joint_group_positions=pos_joint_iniziale;
  joint_group_positions[0]=grad_to_rad(0);
  joint_group_positions[1]=grad_to_rad(-136.929);
  joint_group_positions[2]=grad_to_rad(103.488);
  joint_group_positions[3]=grad_to_rad(33.45);
  joint_group_positions[4]=grad_to_rad(100);
  joint_group_positions[5]=grad_to_rad(90);

  if(!move_to_joints(joint_group_positions)){
    return false;
  }

  if(se_aruco_individuato_aggiorna_array(ID_int))
    return true;

  double rp=20;
  do{
    joint_group_positions[3]=grad_to_rad(33.45)+grad_to_rad(rp);
    if(!move_to_joints(joint_group_positions)){
      return false;
    }
    rp=rp*0.8;
  }while(!success && rp>0);


  if(se_aruco_individuato_aggiorna_array(ID_int))
    return true;
  rp=20;
  do{
    joint_group_positions[3]=grad_to_rad(33.45)-grad_to_rad(rp);
    if(!move_to_joints(joint_group_positions)){
      return false;
    }
    rp=rp*0.8;
  }while(!success && rp>0);



  if(se_aruco_individuato_aggiorna_array(ID_int))
    return true;
}
//ROUTINE 2
  {
    vector<double> joint_group_positions=robot->getCurrentJointValues();
    joint_group_positions=pos_joint_iniziale;
    joint_group_positions[0]=grad_to_rad(-20);
    joint_group_positions[1]=grad_to_rad(-136.929);
    joint_group_positions[2]=grad_to_rad(103.488);
    joint_group_positions[3]=grad_to_rad(33.45);
    joint_group_positions[4]=grad_to_rad(100);
    joint_group_positions[5]=grad_to_rad(90);
    if(!move_to_joints(joint_group_positions)){
      return false;
    }

    if(se_aruco_individuato_aggiorna_array(ID_int))
      return true;

    double rp=20;
    do{
      joint_group_positions[3]=grad_to_rad(33.45)+grad_to_rad(rp);
      if(!move_to_joints(joint_group_positions)){
        return false;
      }
      rp=rp*0.8;
    }while(!success && rp>0);


    if(se_aruco_individuato_aggiorna_array(ID_int))
      return true;

    rp=20;
    do{
      joint_group_positions[3]=grad_to_rad(33.45)-grad_to_rad(rp);
      if(!move_to_joints(joint_group_positions)){
        return false;
      }
      rp=rp*0.8;
    }while(!success && rp>0);


    if(se_aruco_individuato_aggiorna_array(ID_int))
      return true;
  }




  PosizioniBase(str_pos_iniziale);

  if(se_aruco_individuato_aggiorna_array(ID_int))
    return true;

  return false;

}

int routine_aria=1,routine_terra=1;
int num_routine_aria=4,num_routine_terra=1;
bool esplora_tutti_aruco_aria(){
  //IMU DESTINATION,INSP WIND, INSP COVER
    vector<double> joint_group_positions=robot->getCurrentJointValues();

    ROS_INFO("ESPLORO ARUCO PER ARIA, ROUTINE NUMERO: %d",routine_aria);
    blocca_se_vedi_nuovo_aruco();
    if(se_aruco_individuato_aggiorna_array(0)){
      smetti_di_bloccare_se_vedi_nuovo_aruco();
      return true;
    }

    if(routine_aria==1){
      PosizioniBase(str_pos_iniziale_cam_alta);
    }
    if(routine_aria==2){

      joint_group_positions=pos_joint_iniziale_cam_alta;
      joint_group_positions[3]=grad_to_rad(10);
      move_to_joints(joint_group_positions);
    }
    if(routine_aria==3){

      joint_group_positions=pos_joint_iniziale_cam_alta;
      joint_group_positions[3]=grad_to_rad(30);
      move_to_joints(joint_group_positions);
    }
    if(routine_aria==4){

      joint_group_positions=pos_joint_iniziale_cam_alta;
      joint_group_positions[3]=grad_to_rad(-10);
      move_to_joints(joint_group_positions);
    }

    if(se_aruco_individuato_aggiorna_array(0)){
      smetti_di_bloccare_se_vedi_nuovo_aruco();
      return true;
    }


    //Ruota verso i due lati

    {

      if(!Aruco_values[ID_IMU_DESTINATION_PLANE].valid){
        joint_group_positions=robot->getCurrentJointValues();
        joint_group_positions[0]=grad_to_rad(90);
        move_to_joints(joint_group_positions);
      }

      if(se_aruco_individuato_aggiorna_array(0)){
        smetti_di_bloccare_se_vedi_nuovo_aruco();
        return true;
      }

      if(!Aruco_values[ID_INSPECTION_WINDOW_COVER].valid || !Aruco_values[ID_INSPECTION_WINDOW].valid){
        joint_group_positions=robot->getCurrentJointValues();
        joint_group_positions[0]=grad_to_rad(-90);
        move_to_joints(joint_group_positions);
      }

      if(se_aruco_individuato_aggiorna_array(0)){
        smetti_di_bloccare_se_vedi_nuovo_aruco();
        return true;
      }
    }


    routine_aria++;

    //TORNO ALLA POSIZIONE INIZIALE PREDISPOSTO PER NUOVA ROUTINE
    if(routine_aria==2){

      joint_group_positions=robot->getCurrentJointValues();
      joint_group_positions[3]=grad_to_rad(10);
      move_to_joints(joint_group_positions);
    }
    if(routine_aria==3){

      joint_group_positions=robot->getCurrentJointValues();
      joint_group_positions[3]=grad_to_rad(30);
      move_to_joints(joint_group_positions);
    }
    if(routine_aria==4){

      joint_group_positions=robot->getCurrentJointValues();
      joint_group_positions[3]=grad_to_rad(-10);
      move_to_joints(joint_group_positions);


      if(se_aruco_individuato_aggiorna_array(0)){
        smetti_di_bloccare_se_vedi_nuovo_aruco();
        return true;
      }

    }

    smetti_di_bloccare_se_vedi_nuovo_aruco();
    return false;
}
bool esplora_tutti_aruco_per_terra(){
//IMU, Cover storage destination
  vector<double> joint_group_positions=robot->getCurrentJointValues();

  ROS_INFO("ESPLORO ARUCO PER TERRA, ROUTINE NUMERO: %d",routine_terra);

  blocca_se_vedi_nuovo_aruco();

  if(se_aruco_individuato_aggiorna_array(0)){
    smetti_di_bloccare_se_vedi_nuovo_aruco();
    return true;
  }

  if(routine_terra==1){
    PosizioniBase(str_pos_iniziale);

    if(se_aruco_individuato_aggiorna_array(0)){
      smetti_di_bloccare_se_vedi_nuovo_aruco();
      return true;
    }

    PosizioniBase(str_pos_adatta_ricerca_per_terra);
  }

  if(se_aruco_individuato_aggiorna_array(0)){
    smetti_di_bloccare_se_vedi_nuovo_aruco();
    return true;
  }

  //Ruota verso i due lati
  {
    if(!Aruco_values[ID_IMU_MODULE].valid){
      joint_group_positions=pos_joint_adatta_ricerca_per_terra;
      joint_group_positions[0]=grad_to_rad(90);
      move_to_joints(joint_group_positions);
      if(se_aruco_individuato_aggiorna_array(0)){
        smetti_di_bloccare_se_vedi_nuovo_aruco();
        return true;
      }

  }

    if(!Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].valid){
    joint_group_positions=pos_joint_adatta_ricerca_per_terra;
    joint_group_positions[0]=grad_to_rad(-90);
    move_to_joints(joint_group_positions);

    if(se_aruco_individuato_aggiorna_array(0)){
      smetti_di_bloccare_se_vedi_nuovo_aruco();
      return true;
    }
  }
}

  routine_terra++;

  if(routine_terra>num_routine_terra){
    joint_group_positions=robot->getCurrentJointValues();
    joint_group_positions[0]=grad_to_rad(0);
    move_to_joints(joint_group_positions);
    if(se_aruco_individuato_aggiorna_array(0)){
      smetti_di_bloccare_se_vedi_nuovo_aruco();
      return true;
    }
    PosizioniBase(str_pos_iniziale);
    if(se_aruco_individuato_aggiorna_array(0)){
      smetti_di_bloccare_se_vedi_nuovo_aruco();
      return true;
    }
  }

  smetti_di_bloccare_se_vedi_nuovo_aruco();

  return false;
}
bool esplora_tutti_gli_aruco(){

  {
  //Se ho già trovato tutti gli aruco esco
  bool tutti=true;
  for(int i=1;i<aruco_length_array;i++){
    if(!Aruco_values[i].valid)
      tutti=false;
  }
  if(tutti){
    ROS_INFO("Tutti gli aruco sono già stati trovati, NON ESPLORO");
    return true;
  }
  }
  bool ripeti_ciclo=true;
  PosizioniBase(str_pos_iniziale);
  if(!Aruco_values[ID_BUTTON_9].valid){
    esplorazione_middle_panel_per_trovare_aruco("9");
  }
  if(!Aruco_values[ID_BUTTON_1].valid){
    esplorazione_middle_panel_per_trovare_aruco(to_string(ID_BUTTON_1));
  }

  //In questo ciclo verranno cercati gli aruco che si trovano in alto
  ripeti_ciclo=true;
  if(!Aruco_values[ID_IMU_DESTINATION_PLANE].valid || !Aruco_values[ID_INSPECTION_WINDOW].valid|| !Aruco_values[ID_INSPECTION_WINDOW_COVER].valid){

    do{
      esplora_tutti_aruco_aria();

      if(routine_aria>num_routine_aria){
        ripeti_ciclo=false;
      }

      if(Aruco_values[ID_IMU_DESTINATION_PLANE].valid)
        if(Aruco_values[ID_INSPECTION_WINDOW].valid)
          if(Aruco_values[ID_INSPECTION_WINDOW_COVER].valid)
            ripeti_ciclo=false;
    }while(ripeti_ciclo);


  }

  if(!Aruco_values[ID_IMU_MODULE].valid || !Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].valid|| !Aruco_values[ID_INSPECTION_WINDOW_COVER].valid)
  {
    //In questo ciclo verranno cercati gli aruco per terra
    ripeti_ciclo=true;
    do{
      esplora_tutti_aruco_per_terra();

      if(routine_aria>num_routine_terra){
        ripeti_ciclo=false;
      }

      if(Aruco_values[ID_IMU_MODULE].valid)
        if(Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].valid)
          if(Aruco_values[ID_INSPECTION_WINDOW_COVER].valid)
            ripeti_ciclo=false;
    }while(ripeti_ciclo);
  }



  //Nel caso in cui qualche aruco specifico sia sfuggito, faccio partire la sua personale routine

  {

    if(!Aruco_values[ID_BUTTON_1].valid){
      esplorazione_middle_panel_per_trovare_aruco(to_string(ID_BUTTON_1));
    }
    if(!Aruco_values[ID_BUTTON_2].valid){
      esplorazione_middle_panel_per_trovare_aruco(to_string(ID_BUTTON_2));
    }
    if(!Aruco_values[ID_BUTTON_3].valid){
      esplorazione_middle_panel_per_trovare_aruco(to_string(ID_BUTTON_3));
    }
    if(!Aruco_values[ID_BUTTON_4].valid){
      esplorazione_middle_panel_per_trovare_aruco(to_string(ID_BUTTON_4));
    }
    if(!Aruco_values[ID_BUTTON_5].valid){
      esplorazione_middle_panel_per_trovare_aruco(to_string(ID_BUTTON_5));
    }
    if(!Aruco_values[ID_BUTTON_6].valid){
      esplorazione_middle_panel_per_trovare_aruco(to_string(ID_BUTTON_6));
    }
    if(!Aruco_values[ID_BUTTON_7].valid){
      esplorazione_middle_panel_per_trovare_aruco(to_string(ID_BUTTON_7));
    }
    if(!Aruco_values[ID_BUTTON_8].valid){
      esplorazione_middle_panel_per_trovare_aruco(to_string(ID_BUTTON_8));
    }
    if(!Aruco_values[ID_BUTTON_9].valid){
      esplorazione_middle_panel_per_trovare_aruco(to_string(ID_BUTTON_9));
    }
    if(!Aruco_values[ID_IMU_MODULE].valid){
      esplora_cerca_IMU();
    }
    if(!Aruco_values[ID_IMU_DESTINATION_PLANE].valid){
      esplora_cerca_left_panel_IMU();
    }
    if(!Aruco_values[ID_INSPECTION_WINDOW].valid){
      ROS_INFO("ESPLORAZIONE PER ARUCO %d NON ANCORA IMPLEMENTATA",ID_INSPECTION_WINDOW);
    }
    if(!Aruco_values[ID_INSPECTION_WINDOW_COVER].valid){
      esplora_inspection_window_cover();
    }
    if(!Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].valid){
      esplora_inspection_cover_storage();
    }


  }



  bool tutti_quanti_trovati=true;
  for(int i=1;i<aruco_length_array;i++){
    if(!Aruco_values[i].valid){
      tutti_quanti_trovati=false;
      ROS_INFO("DURANTE ESPLORAZIONE COMPLETA, ARUCO %d NON TROVATO",i);
    }
  }

  smetti_di_bloccare_se_vedi_nuovo_aruco();

  if(tutti_quanti_trovati){
    ROS_INFO("DURANTE L' ESPLORAZIONE TUTTI GLI ARUCO SONO STATI TROVATI");
    return true;
  }

  return false;
}

//Panels
/*bool solleva_coperchio(){

  if(!Aruco_values[ID_INSPECTION_WINDOW_COVER].valid){
    if(!esplora_inspection_window_cover())
      return false;
  }

  ROS_INFO("ARUCO TROVATO, POSSO ANDARE A PRENDERE LA COVER");
  Pose p_0_aruco=Aruco_values[ID_INSPECTION_WINDOW_COVER].pose;
  vector<double> joint_group_positions=pos_joint_iniziale;

  joint_group_positions[0]=atan2(p_0_aruco.position.y,p_0_aruco.position.x);

  move_to_joints(joint_group_positions);

  ROS_INFO("VADO IN :POSIZIONE ADATTA PER ANDARE ALLA COVER");

  Affine3d T_aruco_final01,T_aruco_final,T_aruco_final02,T_0_final,T_0_final01,T_0_final02, T_0_aruco;
  Pose pose_final01,pose_final02,pose_final;
  T_0_aruco=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER].pose);


  T_aruco_final.translation().x()=0.0499;//0.05
  T_aruco_final.translation().y()=0.0249;//0.025 - (0.096+0.012+sicurezza)=-0.191 + sicurezza=-0.22
  T_aruco_final.translation().z()=0.18;//0.015
  T_aruco_final.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
  T_0_final=T_0_aruco*T_aruco_final;
  pose_final=homo_to_pose(T_0_final);




  joint_group_positions=pos_joint_iniziale;
  joint_group_positions[0]=atan2(p_0_aruco.position.y,p_0_aruco.position.x);
  joint_group_positions[1]=grad_to_rad(-90);
  joint_group_positions[2]=grad_to_rad(87.4);
  joint_group_positions[3]=grad_to_rad(-78);
  joint_group_positions[4]=grad_to_rad(-90);
  joint_group_positions[5]=grad_to_rad(-120);

  if(!move_to_joints(joint_group_positions)){
    return false;
  }


  pose_final01=robot->getCurrentPose().pose;
  pose_final01.position.z=pose_final.position.z;
  //stampa_Pose(pose_final01);
  if(!move_to_pose_optimized(pose_final01)){
      return false;
  }


  pose_final02=robot->getCurrentPose().pose;
  pose_final02.orientation=pose_final.orientation;
  //stampa_Pose(pose_final02);


  if(!move_to_pose_optimized(pose_final02)){
      return false;
  }

  //stampa_Pose(pose_final);
  if(!move_to_pose_optimized(pose_final)){
      return false;
  }
  if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER)){
    T_0_aruco=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER].pose);
    T_0_final=T_0_aruco*T_aruco_final;
    pose_final=homo_to_pose(T_0_final);
  }

  //scendi per afferrare
  Affine3d T_aruco_final_avvicinato,T_0_final_avvicinato;
  Pose pose_final_avvicinato;

  T_aruco_final_avvicinato.translation().x()=0.0499;//0.05
  T_aruco_final_avvicinato.translation().y()=0.0249;//0.025 - (0.096+0.012+sicurezza)=-0.191 + sicurezza=-0.22
  T_aruco_final_avvicinato.translation().z()=0.1495;//132+35/2=149.5
  T_aruco_final_avvicinato.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
  T_0_final_avvicinato=T_0_aruco*T_aruco_final_avvicinato;
  pose_final_avvicinato=homo_to_pose(T_0_final_avvicinato);


  if(!move_to_pose_optimized(pose_final_avvicinato)){
      return false;
  }

  action_gripper("semi_close");

  ROS_INFO("IL coperchio e' stato afferrato, lo sollevo e lo appoggio nel punto corretto");

  //mi alzo

  if(!move_to_pose_optimized(pose_final)){
      return false;
  }


  ROS_INFO("FUNCTION COMPLETE");
  return true;
}
*/
bool solleva_imu(){
  action_gripper("open");

  if(!Aruco_values[ID_IMU_MODULE].valid){
    if(!esplora_cerca_IMU())
      return false;
  }


  PosizioniBase(str_pos_iniziale);

  Affine3d T_aruco_finalpos,T_0_finalpos_pregrasp,T_0_aruco_IMU_Module;
  Pose pose_final_grasp,pose_final01,pose_final02,pose_final03,pose_final025;
  //calcolo pose

  {
  T_aruco_finalpos.translation().x()=0;
  T_aruco_finalpos.translation().y()=0;
  T_aruco_finalpos.translation().z()=0.1018;//128-(1.2+25)
  T_aruco_finalpos.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI/2,0,0);

  T_0_aruco_IMU_Module=pose_to_homo(Aruco_values[ID_IMU_MODULE].pose);
  T_0_finalpos_pregrasp=T_0_aruco_IMU_Module*T_aruco_finalpos;

  pose_final_grasp=homo_to_pose(T_0_finalpos_pregrasp);
}


  double angolo_disallineamento_imu=atan2(pose_final_grasp.position.y,pose_final_grasp.position.x);

  vector<double> joint_group_positions=pos_joint_iniziale;
  joint_group_positions[0]=angolo_disallineamento_imu;
  move_to_joints(joint_group_positions);

  vector<double> save_joint0=joint_group_positions;

  ROS_INFO("VADO IN :POSIZIONE ADATTA PER ANDARE A PRENDERE IMU");

  joint_group_positions=pos_joint_iniziale;
  joint_group_positions[0]=angolo_disallineamento_imu;
  joint_group_positions[1]=grad_to_rad(-90);
  joint_group_positions[2]=grad_to_rad(87.4);
  joint_group_positions[3]=grad_to_rad(-78);
  joint_group_positions[4]=grad_to_rad(-90);
  joint_group_positions[5]=grad_to_rad(-120);
  move_to_joints(joint_group_positions);

  vector<double> save_joint1=joint_group_positions;

  //sistemo orientamento
  pose_final01=robot->getCurrentPose().pose;
  pose_final01.orientation=pose_final_grasp.orientation;
  if(!move_to_pose_cartesian(pose_final01)){
    if(!move_to_pose(pose_final01,true)){
      return false;
    }
  }


  //sistemo x,y
  pose_final02=robot->getCurrentPose().pose;
  pose_final02.position.x=pose_final_grasp.position.x;
  pose_final02.position.y=pose_final_grasp.position.y;
  if(!move_to_pose_cartesian(pose_final02)){
    if(!move_to_pose(pose_final02,true)){
      return false;
    }
  }
  if(aggiorna_aruco_selezionato(ID_IMU_MODULE)){
    T_0_aruco_IMU_Module=pose_to_homo(Aruco_values[ID_IMU_MODULE].pose);
    T_0_finalpos_pregrasp=T_0_aruco_IMU_Module*T_aruco_finalpos;

    pose_final_grasp=homo_to_pose(T_0_finalpos_pregrasp);
  }


  remove_box("imu_module");

  //mi avvicino alla corretta z
  pose_final025=robot->getCurrentPose().pose;
  pose_final025.position.z=pose_final_grasp.position.z+0.08;
  if(!move_to_pose_optimized(pose_final025)){
      return false;
  }



  //mi avvicino alla corretta z
  pose_final03=robot->getCurrentPose().pose;
  pose_final03.position.z=pose_final_grasp.position.z+0.03;
  //stampa_Pose(pose_final02);
  if(!move_to_pose_cartesian(pose_final03)){
    if(!move_to_pose(pose_final03,true)){
      return false;
    }
  }



//  if(gazebo_bool) {
//    string nome;
//    boost::thread pick_thread(pick, "imu_module",debug[2]);
//  }
    action_gripper("semi_open");

  //Aggiungo collisione imu attached to robot
  {
      Affine3d T_tool_center_of_box,T_0_tool,T_0_imu,T_imu_to_center_of_box,T_0_center_of_box;
      T_0_tool=pose_to_homo(robot->getCurrentPose().pose);
      T_0_imu=pose_to_homo(Aruco_values[ID_IMU_MODULE].pose);

      T_imu_to_center_of_box.translation().x()=0;
      T_imu_to_center_of_box.translation().y()=0;
      T_imu_to_center_of_box.translation().z()=-(0.0012+0.025);
      T_imu_to_center_of_box.linear()=from_rpy_to_rotational_matrix(0,0,0);

      T_0_center_of_box=T_0_imu*T_imu_to_center_of_box;

      T_tool_center_of_box=T_0_tool.inverse()*T_0_center_of_box;



      PoseStamped box_pose;
      float box_size[3];
      string box_name="imu_module";
      box_pose.header.frame_id="ee_link";
      box_size[0]=0.0512;
      box_size[1]=0.165;
      box_size[2]=0.05;


      Pose pose_center_of_box=homo_to_pose(T_tool_center_of_box);
      box_pose.pose=pose_center_of_box;
      SetPoseOrientationRPY(&box_pose.pose,0,0,0);

      collision_boxes[box_name].name=box_name;
      collision_boxes[box_name].pose=box_pose;
      collision_boxes[box_name].size[0]=box_size[0];
      collision_boxes[box_name].size[1]=box_size[1];
      collision_boxes[box_name].size[2]=box_size[2];
      add_and_attach_box(collision_boxes[box_name]);
  }

  //IMU PRESO, MI ALZO

  Pose pose_final_alzato=robot->getCurrentPose().pose;
  pose_final_alzato.position.z+=0.2;

  if(!move_to_pose_optimized(pose_final_alzato)){
    return false;
  }

  return true;
}
bool go_and_attach_imu(double angolo){

    Affine3d T_aruco_final,T_aruco_pre_final,T_0_final,T_0_pre_final;
    Pose pose_final,pose_pre_final;


    T_aruco_pre_final.translation().x()=0.065;
    T_aruco_pre_final.translation().y()=-0.2;
    T_aruco_pre_final.translation().z()=0.2;
    T_aruco_pre_final.linear()=from_rpy_to_rotational_matrix(0,grad_to_rad(90),0)*from_rpy_to_rotational_matrix(grad_to_rad(-90)-grad_to_rad(angolo),0,0);


    T_aruco_final.translation().x()=0.065;
    T_aruco_final.translation().y()=-0.2;
    T_aruco_final.translation().z()=0.15;
    T_aruco_final.linear()=from_rpy_to_rotational_matrix(0,grad_to_rad(90),0)*from_rpy_to_rotational_matrix(grad_to_rad(-90)-grad_to_rad(angolo),0,0);


    T_0_final=pose_to_homo(Aruco_values[ID_IMU_DESTINATION_PLANE].pose)*T_aruco_final;
    T_0_pre_final=pose_to_homo(Aruco_values[ID_IMU_DESTINATION_PLANE].pose)*T_aruco_pre_final;

    pose_final=homo_to_pose(T_0_final);
    pose_pre_final=homo_to_pose(T_0_pre_final);

    if(!allineamento_w_z_x_y(pose_pre_final)){
      if(!move_to_pose_optimized(pose_pre_final))
        return false;
    }

    remove_box("left_panel");
    remove_and_detach_box(collision_boxes["imu_module"]);
    if(!move_to_pose_optimized(pose_final))
      return false;

    action_gripper("open");
    if(gazebo_bool)
      picked=false;

    //Mi allontano tornando a dove ero prima

    if(!move_to_pose_optimized(pose_pre_final))
      return false;

    add_box(collision_boxes["left_panel"]);
    PosizioniBase(str_pos_iniziale);




}
bool solleva_coperchio(){

  if(!Aruco_values[ID_INSPECTION_WINDOW_COVER].valid){
    if(!esplora_inspection_window_cover())
      return false;
  }

  ROS_INFO("ARUCO TROVATO, POSSO ANDARE A PRENDERE LA COVER");
  Pose p_0_aruco=Aruco_values[ID_INSPECTION_WINDOW_COVER].pose;
  vector<double> joint_group_positions=pos_joint_iniziale_cam_alta;

  move_to_joints(joint_group_positions);

  joint_group_positions[0]=atan2(p_0_aruco.position.y,p_0_aruco.position.x);

  move_to_joints(joint_group_positions);

  ROS_INFO("VADO IN :POSIZIONE ADATTA PER ANDARE ALLA COVER");

  Affine3d T_aruco_final01,T_aruco_final,T_aruco_final02,T_0_final,T_0_final01,T_0_final02, T_0_aruco;
  Pose pose_final01,pose_final02,pose_final03,pose_final;
  T_0_aruco=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER].pose);




  T_aruco_final.translation().x()=0.0499;//0.05
  T_aruco_final.translation().y()=0.0249-(0.096+0.0125+0.03);//0.025 - (0.096+0.012+sicurezza)=-0.191 + sicurezza=-0.22
  T_aruco_final.translation().z()=0.017 + 0.05;//0.015
  T_aruco_final.linear()=from_rpy_to_rotational_matrix(0,0,M_PI/2)*from_rpy_to_rotational_matrix(M_PI/2,0,0);
  T_0_final=T_0_aruco*T_aruco_final;
  pose_final=homo_to_pose(T_0_final);



  pose_final01=robot->getCurrentPose().pose;
  pose_final01.orientation=pose_final.orientation;
  //stampa_Pose(pose_final02);


  if(!move_to_pose_optimized(pose_final01)){
      return false;
  }

  pose_final02=robot->getCurrentPose().pose;
  pose_final02.position.z=pose_final.position.z;
  //stampa_Pose(pose_final01);
  if(!move_to_pose_optimized(pose_final02)){
      return false;
  }

  pose_final03=robot->getCurrentPose().pose;
  pose_final03.position.x=pose_final.position.x;
  //stampa_Pose(pose_final01);
  if(!move_to_pose_optimized(pose_final03)){
      return false;
  }


  //stampa_Pose(pose_final);
  if(!move_to_pose_optimized(pose_final)){
      return false;
  }
//  if(se_aruco_individuato_aggiorna_array(ID_INSPECTION_WINDOW_COVER)){
//    T_0_aruco=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER].pose);
//    T_0_final=T_0_aruco*T_aruco_final;
//    pose_final=homo_to_pose(T_0_final);
//  }

  //scendi per afferrare
  Affine3d T_aruco_final_avvicinato,T_0_final_avvicinato;
  Pose pose_final_avvicinato;

  T_aruco_final_avvicinato.translation().x()=0.0499;//0.05
  T_aruco_final_avvicinato.translation().y()=0.0249-(0.096+0.0125+0.03);//0.025 - (0.096+0.012+sicurezza)=-0.191 + sicurezza=-0.22
  T_aruco_final_avvicinato.translation().z()=0.017;//132+35/2=149.5
  T_aruco_final_avvicinato.linear()=from_rpy_to_rotational_matrix(0,0,M_PI/2)*from_rpy_to_rotational_matrix(M_PI/2,0,0);
  T_0_final_avvicinato=T_0_aruco*T_aruco_final_avvicinato;
  pose_final_avvicinato=homo_to_pose(T_0_final_avvicinato);

  remove_box("inspection_box_base_coperchio");
  remove_box("inspection_box_pomello_coperchio");
  sleep(3);
  if(!move_to_pose_optimized(pose_final_avvicinato)){
      return false;
  }

  if(!gazebo_bool)
    action_gripper("semi_close");

  ROS_INFO("IL coperchio e' stato afferrato, lo sollevo e lo appoggio nel punto corretto");


  //aggiungo la collisione del coperchio solidale
  {
    string box_name_coperchio="inspection_box_base_coperchio";
    string box_name_pomello="inspection_box_pomello_coperchio";
    Affine3d T_0_pomello,T_0_coperchio,T_0_tool,T_tool_coperchio,T_tool_pomello;
    Pose pose_pomello_wrt_ee,pose_coperchio_wrt_ee;//pose pomello with respect to end effector.....
    T_0_tool=pose_to_homo(robot->getCurrentPose().pose);
    T_0_pomello=pose_to_homo(collision_boxes[box_name_pomello].pose.pose);
    T_0_coperchio=pose_to_homo(collision_boxes[box_name_coperchio].pose.pose);
    T_tool_coperchio=T_0_tool.inverse()*T_0_coperchio;
    T_tool_pomello=T_0_tool.inverse()*T_0_pomello;

    pose_pomello_wrt_ee=homo_to_pose(T_tool_pomello);
    pose_coperchio_wrt_ee=homo_to_pose(T_tool_coperchio);

    collision_boxes[box_name_coperchio].pose.header.frame_id="ee_link";
    collision_boxes[box_name_pomello].pose.header.frame_id="ee_link";

    collision_boxes[box_name_coperchio].pose.pose=pose_coperchio_wrt_ee;
    collision_boxes[box_name_pomello].pose.pose=pose_pomello_wrt_ee;

    add_and_attach_box(collision_boxes[box_name_coperchio]);
    add_and_attach_box(collision_boxes[box_name_pomello]);

  }



  //mi alzo

  if(!move_to_pose_optimized(pose_final)){
      return false;
  }
  //VADO AD APPOGGIARE COPERCHIO


  Affine3d T_aruco_final_storage,T_aruco_final_storage_vicino,T_0_aruco_storage,T_0_final_storage,T_0_final_storage_vicino;
  Pose pose_final_storage,pose1,pose2,pose_saved1,pose_saved2,pose_saved3,pose_final_storage_vicino;
  T_0_aruco_storage=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].pose);


  T_aruco_final_storage.translation().x()=0.075;
  T_aruco_final_storage.translation().y()=-0.232;
  T_aruco_final_storage.translation().z()=0.1;
  //T_aruco_final_storage.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
  T_aruco_final_storage.linear()=from_rpy_to_rotational_matrix(0,0,M_PI/2)*from_rpy_to_rotational_matrix(M_PI/2,0,0);

  T_0_final_storage=T_0_aruco_storage*T_aruco_final_storage;
  pose_final_storage=homo_to_pose(T_0_final_storage);
/*
  pose_saved1=robot->getCurrentPose().pose;
  pose1=robot->getCurrentPose().pose;
  pose1.position.x=pose_final_storage.position.x;
  pose1.position.y=pose_final_storage.position.y;
  if(!move_to_pose_cartesian(pose1)){
    if(!move_to_pose(pose1,true)){
      return false;
    }
  }

  pose2=robot->getCurrentPose().pose;
  pose2.orientation=pose_final_storage.orientation;
  if(!move_to_pose_cartesian(pose2)){
    if(!move_to_pose(pose2,true)){
      ROS_INFO("MOVIMENTO FALLITO");
      return false;
    }
  }

  //movimento pre finale
  if(!move_to_pose_cartesian(pose_final_storage)){
    if(!move_to_pose(pose1,true)){
      return false;
    }
  }
  */

  if(!allineamento_x_y_w_z(pose_final_storage)){
    if(!move_to_pose_optimized(pose_final_storage))
      return false;
  }

  action_gripper("open");
  remove_and_detach_box(collision_boxes["inspection_box_base_coperchio"]);
  remove_and_detach_box(collision_boxes["inspection_box_pomello_coperchio"]);

  ROS_INFO("FUNCTION COMPLETE");
  return true;
}
bool left_panel(){

  if(!Aruco_values[ID_IMU_DESTINATION_PLANE].valid){
    if(!esplora_cerca_left_panel_IMU())
      ROS_INFO("LEFT PANEL NON TROVATO");
  }

  solleva_imu();


//  sleep(5);
//  picked=false;
//  if(!move_to_pose_cartesian(pose_final_pose_pregrasp)){
//    if(!move_to_pose(pose_final_pose_pregrasp,true))
//      return false;
//  }


//  sleep(2);

//  T_aruco_finalpos.translation().z()=0.103;


//  T_0_finalpos_pregrasp=T_0_aruco_IMU_Module*T_aruco_finalpos;

//  Pose pose_final_pose=homo_to_pose(T_0_finalpos_pregrasp);

//  if(!move_to_pose_cartesian(pose_final_pose)){
//    if(!move_to_pose(pose_final_pose,true))
//      return false;
//  }



//  T_aruco_finalpos.translation().z()=0.13;


//  T_0_finalpos_pregrasp=T_0_aruco_IMU_Module*T_aruco_finalpos;

//  pose_final_pose=homo_to_pose(T_0_finalpos_pregrasp);

//  if(!move_to_pose_cartesian(pose_final_pose)){
//    if(!move_to_pose(pose_final_pose,true))
//      return false;
//  }


//  PosizioniBase(str_pos_iniziale_cam_alta);

//  if(!move_to_pose_cartesian(pose_final_pose)){
//    if(!move_to_pose(pose_final_pose,true))
//       return false;
//  }


//  action_gripper("open");


  PosizioniBase(str_pos_iniziale);


  return true;
}
bool action_aruco_button(string ID_str){
  stringstream ss;
  int ID_int;

  ss<<ID_str;
  ss >> ID_int;

  if(!Aruco_values[ID_int].valid){//se non è mai stato trovato l'aruco di interesse

    if(!esplorazione_middle_panel_per_trovare_aruco(ID_str)){//qui inizia a cercare l'aruco
        ROS_INFO("ARUCO NON TROVATO, ID:%d",ID_int);
      return false;
    }

  }
  ROS_INFO("Aruco trovato, vado a premere il pulsante");


  PosizioniBase(str_pos_iniziale_cam_alta);
  action_gripper("close");



  Affine3d T_aruco_finalpos,T_0_finalpos_pregrasp, T_0_aruco;

  T_0_aruco=pose_to_homo(Aruco_values[ID_int].pose);


  //stampa_Pose(homo_to_pose(T_0_aruco));


  T_aruco_finalpos.translation().x()=0;
  T_aruco_finalpos.translation().y()=-0.055; //-0.055 per la gara
  T_aruco_finalpos.translation().z()=0.20; //23+135+12,5+sicurezza=170,5mm + sicurezza=0.1705 metri + 0.03=0.20
  T_aruco_finalpos.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0);//*from_rpy_to_rotational_matrix(M_PI,0,0);


  T_0_finalpos_pregrasp=T_0_aruco*T_aruco_finalpos;
  //In questo punto ho T_0_final

  Pose pose_final_pose_pregrasp=homo_to_pose(T_0_finalpos_pregrasp);

  //AVVICINAMENTO
  Pose pose1,pose2,pose3;
  {
    pose1=robot->getCurrentPose().pose;
    pose1.orientation=pose_final_pose_pregrasp.orientation;
    if(!move_to_pose_optimized(pose1)){
        return false;
    }

    pose2=robot->getCurrentPose().pose;
    pose2.position.x=pose_final_pose_pregrasp.position.x;
    pose2.position.y=pose_final_pose_pregrasp.position.y;
    if(!move_to_pose_optimized(pose2)){
        return false;
    }

    pose3=robot->getCurrentPose().pose;
    pose3.position.z=pose_final_pose_pregrasp.position.z;
    if(!move_to_pose_optimized(pose3)){
        return false;
    }


  }



  remove_box("button_"+ID_str);
  remove_box("middle_panel");
  //remove_box("gripper_box");


  if(!move_to_pose_optimized(pose_final_pose_pregrasp)){
      return false;
  }




  //POSIZIONE PER PREMERE PULSANTE
  T_aruco_finalpos.translation().x()=0;
  T_aruco_finalpos.translation().y()=-0.055; //-0.055 per la gara
  T_aruco_finalpos.translation().z()=0.16;//0.18 È GIUSTO //17+135+12,5=164.5 mm=0.164m    - in teoria 1mm di spessore aruco=0.163
  T_aruco_finalpos.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0);//*from_rpy_to_rotational_matrix(M_PI,0,0);


  Affine3d T_0_finalpos_premuto=T_0_aruco*T_aruco_finalpos;
  //In questo punto ho T_0_final

  Pose pose_final_pose_premuto=homo_to_pose(T_0_finalpos_premuto);

  ros::NodeHandle n;
  n.setParam("freno_a_mano_buttons",true);

  move_to_pose_optimized(pose_final_pose_premuto);
  sleep(2);


  n.setParam("freno_a_mano_buttons",false);

  //RITORNO INDIETRO-------------------


  if(!move_to_pose_optimized(pose_final_pose_pregrasp)){
    if(!move_to_pose(pose_final_pose_premuto,true))
      return false;
  }

  add_box(collision_boxes["button_"+ID_str]);
  add_box(collision_boxes["middle_panel"]);
  //add_and_attach_box(collision_boxes["gripper_box"]);

  if(!move_to_pose_optimized(pose3)){
    if(!move_to_pose(pose_final_pose_premuto,true))
      return false;
  }

  if(!move_to_pose_optimized(pose2)){
    if(!move_to_pose(pose_final_pose_premuto,true))
      return false;
  }

  if(!move_to_pose_optimized(pose1)){
    if(!move_to_pose(pose_final_pose_premuto,true))
      return false;
  }


  PosizioniBase(str_pos_iniziale_cam_alta);

  //add_box(collision_boxes["button_"+ID_str]);
  action_gripper("open");

  return true;
}
bool right_panel(){
  action_gripper("open");

  //Se necessario fa le esplorazioni
  if(!Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].valid){
    if(!esplora_inspection_cover_storage())
      return false;
  }
  if(!Aruco_values[ID_INSPECTION_WINDOW_COVER].valid){
    if(!esplora_inspection_window_cover())
      return false;
  }


  if(!solleva_coperchio()) return false;

  Affine3d T_aruco_final_storage,T_aruco_final_storage_vicino,T_0_aruco_storage,T_0_final_storage,T_0_final_storage_vicino;
  Pose pose_final_storage,pose1,pose2,pose_saved1,pose_saved2,pose_saved3,pose_final_storage_vicino;
  T_0_aruco_storage=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].pose);


  T_aruco_final_storage.translation().x()=0;
  T_aruco_final_storage.translation().y()=0;
  T_aruco_final_storage.translation().z()=0.2;
  T_aruco_final_storage.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
  T_0_final_storage=T_0_aruco_storage*T_aruco_final_storage;
  pose_final_storage=homo_to_pose(T_0_final_storage);

  pose_saved1=robot->getCurrentPose().pose;
  pose1=robot->getCurrentPose().pose;
  pose1.position.x=pose_final_storage.position.x;
  pose1.position.y=pose_final_storage.position.y;
  if(!move_to_pose_cartesian(pose1)){
    if(!move_to_pose(pose1,true)){
      return false;
    }
  }

  pose2=robot->getCurrentPose().pose;
  pose2.orientation=pose_final_storage.orientation;
  if(!move_to_pose_cartesian(pose2)){
    if(!move_to_pose(pose2,true)){
      ROS_INFO("MOVIMENTO FALLITO");
      return false;
    }
  }

  //movimento pre finale
  if(!move_to_pose_cartesian(pose_final_storage)){
    if(!move_to_pose(pose1,true)){
      return false;
    }
  }

  if(!gazebo_bool){
    T_aruco_final_storage_vicino.translation().x()=0;
    T_aruco_final_storage_vicino.translation().y()=0;
    T_aruco_final_storage_vicino.translation().z()=0.05;
    T_aruco_final_storage_vicino.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
    T_0_final_storage_vicino=T_0_aruco_storage*T_aruco_final_storage_vicino;
    pose_final_storage_vicino=homo_to_pose(T_0_final_storage_vicino);

    if(!move_to_pose_cartesian(pose_final_storage_vicino)){
      if(!move_to_pose(pose_final_storage_vicino,true)){
        return false;
      }
    }
  }
  action_gripper("open");
  //TORNIAMO INDIETRO
  if(!move_to_pose_cartesian(pose2)){
    if(!move_to_pose(pose2,true)){
      return false;
    }
  }

  if(!move_to_pose_cartesian(pose1)){
    if(!move_to_pose(pose1,true)){
      return false;
    }
  }

  if(!move_to_pose_cartesian(pose_saved1)){
    if(!move_to_pose(pose_saved1,true)){
      return false;
    }
  }

  int cont=0;
  int aruco_nascosto_trovato=-1;
  ur3_control::aruco_serviceResponse msg=bridge_service(str_md_rd,"");
  for(int i=0;i<msg.all_aruco_found.size();i++){
    if(msg.all_aruco_found[i]==true){
      cont++;
      if(i>0 && i<10){
        ROS_INFO("ARUCO NASCOSTO TROVATO, ID: %d",i);
        aruco_nascosto_trovato=i;
      }
    }
}
  if(aruco_nascosto_trovato!=-1){
    //DEVE PRIMA RITORNARE INDIETRO E POI PREMERE IL PULSANTE. MANCA LA PARTE IN CUI TORNA INDIETRO
    action_aruco_button(to_string(aruco_nascosto_trovato));

  }
  else{
    ROS_INFO("Vedo %d elementi ma nessun elemento corrispondente ai pulsanti da premere",cont);
    //devo cambiare posizione
  }
  return true;


{
//  Affine_valid T_0_aruco_valid_panel=homo_0_aruco_elaration();
//  if(! T_0_aruco_valid_panel.valid){
//    //Devo cambiare posizione

//    return false;
//  }



//  //ARUCO 13 TROVATO


//  Affine3d T_aruco_finalpos_panel,T_0_finalpos_panel, T_0_aruco_panel;
//  Pose pose_final_pose_panel;
//  T_0_aruco_panel=T_0_aruco_valid_panel.homo_matrix;


//  T_aruco_finalpos_panel.translation().x()=0.05;//0.05
//  T_aruco_finalpos_panel.translation().y()=-0.22;//0.025 - (0.096+0.012+sicurezza)=-0.191 + sicurezza=-0.22
//  T_aruco_finalpos_panel.translation().z()=0.015;//0.015
//  //T_aruco_finalpos_panel.linear()=from_rpy_to_rotational_matrix(0,0,M_PI/2)*from_rpy_to_rotational_matrix(-M_PI/2,0,0);
//  T_aruco_finalpos_panel.linear()=from_rpy_to_rotational_matrix(0,0,M_PI/2)*from_rpy_to_rotational_matrix(M_PI/2,0,0);
//  T_0_finalpos_panel=T_0_aruco_panel*T_aruco_finalpos_panel;
//  pose_final_pose_panel=homo_to_pose(T_0_finalpos_panel);
//  stampa_Pose(pose_final_pose_panel);

//  //CONOSCO LA MIA POSIZIONE PER RAGGIUNGERE IL BLOCCO
//  //CERCO DI AVVICINARMI
//  vector<double> joint_group_positions;
//  double alpha=atan2(pose_final_pose_panel.position.y,pose_final_pose_panel.position.x);
//  joint_group_positions=pos_joint_iniziale_cam_alta;
//  joint_group_positions[0] = alpha;  // radians
//  robot->setJointValueTarget(joint_group_positions);
//  success = (robot->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//  ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
//  robot->move();


//  //Ora dovrei essere orientato correttamente, ora mi posiziono con la corretta 'z'


//  Pose p2=robot->getCurrentPose().pose;
//  p2.position.z=pose_final_pose_panel.position.z;
//  if(!move_to_pose_cartesian(p2)){
//    move_to_pose(p2,true);
//  }


//  //ORA POSSO FINALMENTE ANDARE AL INSPECTION WINDOW


//  if(!move_to_pose_cartesian(pose_final_pose_panel)){
//    move_to_pose(pose_final_pose_panel,true);
//  }


//  //MI AVVICINO ALL INSPECTION

//  T_aruco_finalpos_panel.translation().y()=-0.107;//0.025 - (0.132)=-0.107
//  T_0_finalpos_panel=T_0_aruco_panel*T_aruco_finalpos_panel;
//  pose_final_pose_panel=homo_to_pose(T_0_finalpos_panel);
//  stampa_Pose(pose_final_pose_panel);
//  move_to_pose_cartesian(pose_final_pose_panel);

//  sleep(1);
//  action_gripper("semi_close");

//  //AFFERRO


//  T_aruco_finalpos_panel.translation().z()=0.06;
//  T_0_finalpos_panel=T_0_aruco_panel*T_aruco_finalpos_panel;
//  pose_final_pose_panel=homo_to_pose(T_0_finalpos_panel);
//  stampa_Pose(pose_final_pose_panel);
//  move_to_pose_cartesian(pose_final_pose_panel);


//  T_aruco_finalpos_panel.translation().y()=-0.22;
//  T_0_finalpos_panel=T_0_aruco_panel*T_aruco_finalpos_panel;
//  pose_final_pose_panel=homo_to_pose(T_0_finalpos_panel);
//  stampa_Pose(pose_final_pose_panel);
//  move_to_pose_cartesian(pose_final_pose_panel);


//  //PRE RAGGIUNGIMENTO INSPECTION COVER STORAGE

//  {
//    vector<double> joint_group_positions=pos_joint_iniziale;
//    joint_group_positions[0] = joint_group_positions[0]- grad_to_rad(35);  // radians
//    joint_group_positions[3] = grad_to_rad(70);  // radians
//    joint_group_positions[5] = grad_to_rad(0);  // radians
//    robot->setJointValueTarget(joint_group_positions);
//    success = (robot->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//    ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
//    robot->move();
//  }
///*
//  bridge_service(str_md_next_aruco,"14");
//  sleep(2);
//  ruota_e_cerca_aruco();
//  sleep(2);//ATTENTO UNO SLEEP, CREDO SIA UTILE PERCHÈ SENNÒ IL BLOCCO SULLA ROTAZIONE INFLUISCE SUL PROSSIMO MOVIMENTO


//  Affine_valid T_0_aruco_valid_ground=homo_0_aruco_elaration();
//  if(! T_0_aruco_valid_ground.valid){
//    //Devo cambiare posizione

//    return false;
//  }


//  Affine3d T_aruco_finalpos_ground,T_0_finalpos_ground, T_0_aruco_ground;
//  T_0_aruco_ground=T_0_aruco_valid_ground.homo_matrix;
//*/
//  Affine3d T_aruco_finalpos_ground,T_0_finalpos_ground, T_0_aruco_ground;
//  Pose pose_final_pose_ground;
//  T_0_aruco_ground=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER_STORAGE].pose);

//  T_aruco_finalpos_ground.translation().x()=0;
//  T_aruco_finalpos_ground.translation().y()=0;
//  T_aruco_finalpos_ground.translation().z()=0.2;
//  T_aruco_finalpos_ground.linear()=from_rpy_to_rotational_matrix(0,M_PI/2,0)*from_rpy_to_rotational_matrix(M_PI,0,0);
//  T_0_finalpos_ground=T_0_aruco_ground*T_aruco_finalpos_ground;
//  pose_final_pose_ground=homo_to_pose(T_0_finalpos_ground);
//  stampa_Pose(pose_final_pose_ground);
//  stampa_homo(T_aruco_finalpos_ground);

//  if(!move_to_pose_cartesian(pose_final_pose_ground)){
//    move_to_pose(pose_final_pose_ground,true);
//  }


//  action_gripper("open");


//  PosizioniBase(str_pos_iniziale);
//  action_gripper("open");



//  return true;
}

}
bool go_and_recognize_id_in_inspection_window(){

  PosizioniBase(str_pos_iniziale_cam_alta);
  action_gripper("open");

  Affine3d T_0_scatola,T_scatola_final,T_0_final;
  T_0_scatola=pose_to_homo(Aruco_values[ID_INSPECTION_WINDOW_COVER].pose);

  T_scatola_final.translation().x()=0.05;
  T_scatola_final.translation().y()=-0.07;
  T_scatola_final.translation().z()=0.07;
  T_scatola_final.linear()=from_rpy_to_rotational_matrix(0,grad_to_rad(90),0);

  T_0_final=T_0_scatola*T_scatola_final;

  remove_box("inspection_box_base_coperchio");
  remove_box("inspection_box_pomello_coperchio");
  if(!move_to_pose_optimized(homo_to_pose(T_0_final))){

    return false;

  }


  int cont=0;
  int aruco_nascosto_trovato=-1;
  ur3_control::aruco_serviceResponse msg=bridge_service(str_md_rd,"");
  for(int i=0;i<msg.all_aruco_found.size();i++){
    if(msg.all_aruco_found[i]==true){
      cont++;
      if(i>0 && i<10){
        ROS_INFO("ARUCO NASCOSTO TROVATO, ID: %d",i);
        aruco_nascosto_trovato=i;
      }
    }
}
  if(aruco_nascosto_trovato!=-1){
    //DEVE PRIMA RITORNARE INDIETRO E POI PREMERE IL PULSANTE. MANCA LA PARTE IN CUI TORNA INDIETRO

    ros::NodeHandle n;
    n.setParam("hidden_id",to_string(aruco_nascosto_trovato));
  }
  else{
    ROS_INFO("Vedo %d elementi ma nessun elemento corrispondente ai pulsanti da premere",cont);
    //devo cambiare posizione
  }
  return true;

}

//Inizializations
void prova(){
  /*Collision_box_type b;
  b.name="TCP";
  b.pose.header.frame_id="ee_link";
  b.pose.pose.position.x=0.096;
  b.pose.pose.position.y=0;
  b.pose.pose.position.z=0;
  b.size[0]=0.001;
  b.size[1]=0.001;
  b.size[2]=0.001;
  add_box(b);


  sleep(10);

  b.name="TCP2";
  b.pose.header.frame_id="ee_link";
  b.pose.pose.position.x=0.128;
  b.pose.pose.position.y=0;
  b.pose.pose.position.z=0;
  b.size[0]=0.001;
  b.size[1]=0.001;
  b.size[2]=0.001;
  add_box(b);
*/

//  string nome;
//  boost::thread pick_thread(pick, "imu_module",debug[2]);
//  action_gripper("semi_open");
//  sleep(10);
//  picked=false;

}
void initialize_parameters(){
  show_log=false;
  adding_collision_enabled=true;
  robot->setPlannerId("RRTConnectkConfigDefault");
  robot->setPlanningTime(5);
  pose_pannello_elaborata.valid=false;

  ros::NodeHandle n;
  n.setParam("freno_a_mano_buttons",false);
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
    pos_joint_iniziale=robot->getCurrentJointValues();
    pos_joint_adatta_ricerca_per_terra=robot->getCurrentJointValues();
    debug=robot->getCurrentJointValues();
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
            double j[6];
            for(int i=0;i<6;i++){
                inFile>>j[i];
                //printf("%d:%f ",i,j[i]);
            }
            //printf("\n");

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
            if(nome==str_pos_iniziale){
              for (int i=0;i<6;i++) {
                  pos_joint_iniziale[i]=grad_to_rad(j[i]);
              }
          }
            if(nome=="debug"){
              for (int i=0;i<6;i++) {
                  debug[i]=(double)j[i];
              }
          }
            if(nome==str_pos_adatta_ricerca_per_terra){
              for (int i=0;i<6;i++) {
                  pos_joint_adatta_ricerca_per_terra[i]=grad_to_rad(j[i]);
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
                //printf("rpy: %d %d %d\n",pos_rpy_ta[0],pos_rpy_ta[1],pos_rpy_ta[2]);
            }
            if(nome==str_tb){

                pos_rpy_tb[0]=r0;
                pos_rpy_tb[1]=p0;
                pos_rpy_tb[2]=y0;

                //printf("rpy: %d %d %d\n",pos_rpy_tb[0],pos_rpy_tb[1],pos_rpy_tb[2]);
                }

        }

    }

    inFile.close();
    pos_joint_iniziale_cam_alta=pos_joint_iniziale;
    pos_joint_iniziale_cam_alta[5]=grad_to_rad(-90);
}
void set_homo_std_matrix(){

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


  Vector3d translation_camera_camera_gazebo(0,0,0);
  Matrix3d rotation_camera_camera_gazebo;
  Vector3d xvec_cc(0,0,1),yvec_cc(0,-1,0),zvec_cc(1,0,0);
  rotation_camera_camera_gazebo.col(0)=xvec_cc;
  rotation_camera_camera_gazebo.col(1)=yvec_cc;
  rotation_camera_camera_gazebo.col(2)=zvec_cc;
  T_camera_camera_gazebo.translation()=translation_camera_camera_gazebo;
  T_camera_camera_gazebo.linear()=rotation_camera_camera_gazebo;


  T_tool_camera_gazebo=T_tool_camera*T_camera_camera_gazebo;

}
void add_initial_collision_environment(){
  ROS_INFO("ADDING std environment collision");
  PoseStamped box_pose;
  float box_size_rialzo[3],box_size_table[3];
  string box_name;

  //Impalcatura
  {
  box_name="impalcatura";
  box_size_rialzo[0]=2;//0.4 è il massimo
  box_size_rialzo[1]=0.22;
  box_size_rialzo[2]=0.14;

  box_pose.header.frame_id="base_link";
  box_pose.pose.position.x=box_size_rialzo[0]/2;
  box_pose.pose.position.y=0;
  box_pose.pose.position.z=-box_size_rialzo[2]/2;

  collision_boxes[box_name].name=box_name;
  collision_boxes[box_name].pose=box_pose;
  collision_boxes[box_name].size[0]=box_size_rialzo[0];
  collision_boxes[box_name].size[1]=box_size_rialzo[1];
  collision_boxes[box_name].size[2]=box_size_rialzo[2];
  add_box(collision_boxes[box_name]);

  }

  //ikea_table
  {
  box_name="ikea_table";
  box_size_table[0]=0.75;
  box_size_table[1]=0.75;
  box_size_table[2]=0.001;

  box_pose.header.frame_id="base_link";
  box_pose.pose.position.x=0;
  box_pose.pose.position.y=0;
  box_pose.pose.position.z=-(box_size_rialzo[2] + box_size_table[2]/2);

  collision_boxes[box_name].name=box_name;
  collision_boxes[box_name].pose=box_pose;
  collision_boxes[box_name].size[0]=box_size_table[0];
  collision_boxes[box_name].size[1]=box_size_table[1];
  collision_boxes[box_name].size[2]=box_size_table[2];
  add_box(collision_boxes[box_name]);
  }

  //Camera_collision
  {
    /*Diversamente dalle altre funzioni, qui verrà fatto tutto rispetto al tool. Questo per mantenere solidale la box*/
    Affine3d T_camera_to_center_of_box;

    //T_0_camera=pose_to_homo(robot->getCurrentPose().pose) * T_tool_camera_gazebo;


    //altezza camera: 76.95/sin(80)=75.78   assumo distanza tra camera centro camera e fine camera=11.4


    T_camera_to_center_of_box.translation().x()=-0.02649;
    T_camera_to_center_of_box.translation().y()=0;
    T_camera_to_center_of_box.translation().z()=-0.02293/2;
    T_camera_to_center_of_box.linear()=from_rpy_to_rotational_matrix(0,0,0);//non mi interessa cambiare la rotazione


    Affine3d T_tool_center_of_box=T_tool_camera_gazebo*T_camera_to_center_of_box;
    Pose pose_center_of_box=homo_to_pose(T_tool_center_of_box);


    PoseStamped box_pose;
    float box_size[3];
    string box_name="camera_box";

    box_size[0]=0.1;//thickness
    box_size[1]=0.055;//width
    box_size[2]=0.03;//height

    box_pose.header.frame_id="ee_link";

    box_pose.pose=pose_center_of_box;

    collision_boxes[box_name].name=box_name;
    collision_boxes[box_name].pose=box_pose;
    collision_boxes[box_name].size[0]=box_size[0];
    collision_boxes[box_name].size[1]=box_size[1];
    collision_boxes[box_name].size[2]=box_size[2];


    add_box(collision_boxes[box_name]);
    attach_box(collision_boxes[box_name]);
  }

  //Gripper_collision
  {

    update_gripper_collision_box("open");


  }
}
bool load_aruco_values_from_txt(){

  ifstream inFile;
  string nome,tipo;
  string pkgpath = ros::package::getPath("ur3_control");
  string path_txt="/txt/aruco_values.txt";
  string path_total=pkgpath + path_txt;
  cout<<"percorso txt: "<<path_total<<endl<<endl;
  inFile.open(path_total);
  if (!inFile) {
      cerr << "Unable to open file aruco_values.txt";
      return false;  // call system to stop
  }

  for(int i=1;i<aruco_length_array;i++){

    inFile.read((char*)(&Aruco_values[i]),sizeof(Aruco_values[i]));
    if(Aruco_values[i].valid){
      ROS_INFO("Sono a conoscenza dell'aruco %d:",i);
      se_aruco_individuato_add_collision(i);
    }
  }


  inFile.close();

  return true;

}
void ALL_INITIAL_VOIDS(){
  ros::NodeHandle n;
  bool load_aruco=false;
  n.getParam("load_aruco",load_aruco);
  n.getParam("gazebox",gazebo_bool);
  initialize_parameters();
  set_homo_std_matrix();
  load_parameters();
  calibrazione_gripper();

  if(gazebo_bool){
    PosizioniBase(str_pos_iniziale);
  }


  add_initial_collision_environment();


  if(load_aruco){
    load_aruco_values_from_txt();
  }


  prova();
}
int sss;
