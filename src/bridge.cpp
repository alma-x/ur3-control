#include "ros/ros.h"
#include "ros/service.h"
#include "iostream"
#include "stdio.h"
#include "ur3_control/aruco_service.h"
#include "ur3_control/cv_to_bridge.h"
#include "actionlib_msgs/GoalID.h"
#include "string.h"
#include "library.h"
#include "ur3_control/cv_server.h"
#include "std_msgs/Bool.h"
using namespace std;
ur3_control::cv_to_bridge msg_from_cv;
sensor_msgs::JointState msg_from_joints;
ros::ServiceClient client;
bool bool_md_bpa=false;
bool bool_blocca_per_ogni_nuovo_aruco=false;
bool aruco_trovati[aruco_length_array];
int ID_DA_BLOCCARE=-1;
void stampa_cv_msg(const ur3_control::cv_to_bridge msg){
ROS_INFO("Message received:\n x:%f \n y:%f \n z:%f \n success:%s",msg.x,msg.y,msg.z,(msg.success)? "success":"not success");
}
void cv_callback(const ur3_control::cv_to_bridge& msg){

  msg_from_cv=msg;


  if(bool_md_bpa && msg_from_cv.success && msg.id_aruco==ID_DA_BLOCCARE){
    ROS_INFO("BRIDGE:BLOCCO ROBOT");
    bool_md_bpa=false;

//    actionlib_msgs::GoalID msg_traj_cancel;
//    msg_traj_cancel.id="";
//    pub_traj_cancel.publish(msg_traj_cancel);
    ROS_INFO("Aruco trovato, traiettoria cancellata");
    robot->stop();
  }

  if(bool_blocca_per_ogni_nuovo_aruco && msg.aruco_found.size()!=0){
    for (int i=0;i<aruco_length_array;i++) {
      if(!aruco_trovati[i] && msg.aruco_found[i]){

        bool_blocca_per_ogni_nuovo_aruco=false;
        ROS_INFO("Aruco %d trovato, traiettoria cancellata",i);
        robot->stop();
      }

    }
  }


}
void joint_callback(const sensor_msgs::JointState& msg){

  msg_from_joints=msg;
}
bool callback_modality(ur3_control::aruco_service::Request &req, ur3_control::aruco_service::Response &res){


  if(show_log)
     ROS_INFO("Bridge:Chiamata ricevuta:\nModalita:%s\n\n",req.modality.c_str());

  if(str_md_bpa==req.modality){
    ID_DA_BLOCCARE=msg_from_cv.id_aruco;
    ROS_INFO("BRIDGE: BLOCCO APPENA VEDO ARUCO %d",ID_DA_BLOCCARE);
    bool_md_bpa=true;
  }
  if(str_md_stop_bpa==req.modality){
    ID_DA_BLOCCARE=-1;
    bool_md_bpa=false;
  }
  if(str_md_next_aruco==req.modality){

      ur3_control::cv_server cv_service_msg;
      cv_service_msg.request.next_aruco=true;
      cv_service_msg.request.message="select_next_aruco";
      cv_service_msg.request.second_information=req.second_information;
      client.call(cv_service_msg);

      res.moreTargets=cv_service_msg.response.moreTargets;
      if(show_log)
        ROS_INFO("targets remaining:%d",res.moreTargets);
  }
  if("exit"==req.modality){
      bool_exit=true;
      ur3_control::cv_server cv_service_msg;
      cv_service_msg.request.message="exit";
      client.call(cv_service_msg);
  }
  if("stop_trajectory"==req.modality){

//    actionlib_msgs::GoalID msg_traj_cancel;
//    msg_traj_cancel.id="s";
//    pub_traj_cancel.publish(msg_traj_cancel);
//    ROS_INFO("traiettoria cancellata");
    robot->stop();

  }
  if("blocca_se_vedi_nuovo_aruco"==req.modality){
    ROS_INFO("BLOCCHERO' LA PRIMA VOLTA CHE TROVERO' UN NUOVO ARUCO");
    bool_blocca_per_ogni_nuovo_aruco=true;
    for (int i=0;i<aruco_length_array;i++) {
      aruco_trovati[i]=req.aruco_trovati[i];
    }
  }
  if("smetti_di_bloccare_se_vedi_nuovo_aruco"==req.modality){
    bool_blocca_per_ogni_nuovo_aruco=false;
  }
  if("turn_on_off_camera"==req.modality){
      
      ur3_control::cv_server cv_service_msg;
      cv_service_msg.request.next_aruco=false;
      cv_service_msg.request.message="turn_on_off_camera";
      cv_service_msg.request.second_information="";
      client.call(cv_service_msg);

      res.moreTargets=cv_service_msg.response.moreTargets;
      if(show_log)
        ROS_INFO("targets remaining:%d",res.moreTargets);

  }  
  res.aruco_found=msg_from_cv.success;
  res.x=msg_from_cv.x;
  res.y=msg_from_cv.y;
  res.z=msg_from_cv.z;
  res.vector=msg_from_cv.vector;
  res.id_aruco=msg_from_cv.id_aruco;
  res.finger_joint=float(msg_from_joints.position[1]);
  res.all_aruco_found=msg_from_cv.aruco_found;
  return true;
}

bool freno_a_mano_is_on=false;
bool button_was_pushed=false;
void button_callback(const std_msgs::Bool &msg){

  ros::NodeHandle n;
  bool button_need_control;
  n.getParam("freno_a_mano_buttons",button_need_control);
  bool button_msg=msg.data;
  if(button_msg && button_need_control ){

    n.setParam("freno_a_mano_buttons",false);
    ROS_INFO("RILEVATO ARUCO PREMUTO TRAMITE TOPIC");
//    freno_a_mano_is_on=true;
    robot->stop();
  }
}

int main(int argc, char** argv){

  signal(SIGINT, signal_callback_handler);
  ros::init(argc, argv, "bridge");


  static ros::AsyncSpinner spinner(1);

  ros::Subscriber sub,sub_joint,sub_but1,sub_but2,sub_but3,sub_but4,sub_but5,sub_but6,sub_but7,sub_but8,sub_but9;
  ros::NodeHandle n;
  ros::ServiceServer serv;
  pub_traj_cancel = n.advertise<actionlib_msgs::GoalID>("/arm_controller/follow_joint_trajectory/cancel", 100);
  sub=n.subscribe("/aruco_bridge_opencv",1,cv_callback);
  sub_joint=n.subscribe("/joint_states",1,joint_callback);
  sub_but1=n.subscribe("button1",1,button_callback);
  sub_but2=n.subscribe("button2",1,button_callback);
  sub_but3=n.subscribe("button3",1,button_callback);
  sub_but4=n.subscribe("button4",1,button_callback);
  sub_but5=n.subscribe("button5",1,button_callback);
  sub_but6=n.subscribe("button6",1,button_callback);
  sub_but7=n.subscribe("button7",1,button_callback);
  sub_but8=n.subscribe("button8",1,button_callback);
  sub_but9=n.subscribe("button9",1,button_callback);

  serv=n.advertiseService("aruco_modality", callback_modality);
  client = n.serviceClient<ur3_control::cv_server>("/cv_server");
  MoveGroupInterface move_group(PLANNING_GROUP);

  robot=&move_group;
  msg_from_cv.success=false;
  bool_md_bpa=false;

  while(ros::ok() && !bool_exit){
  ros::spinOnce();
  }
  return 0;
}
