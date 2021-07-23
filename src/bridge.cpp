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
using namespace std;
ur3_control::cv_to_bridge msg_from_cv;
sensor_msgs::JointState msg_from_joints;
ros::ServiceClient client;
bool bool_md_bpa=false;
ros::Publisher pub_traj_cancel;
void stampa_cv_msg(const ur3_control::cv_to_bridge msg){
ROS_INFO("Message received:\n x:%f \n y:%f \n z:%f \n success:%s",msg.x,msg.y,msg.z,(msg.success)? "success":"not success");
}
void cv_callback(const ur3_control::cv_to_bridge& msg){

  msg_from_cv=msg;


  if(bool_md_bpa && msg_from_cv.success){

    bool_md_bpa=false;

    actionlib_msgs::GoalID msg_traj_cancel;
    msg_traj_cancel.id="";
    pub_traj_cancel.publish(msg_traj_cancel);
    ROS_INFO("Aruco trovato, traiettoria cancellata");
//    robot->stop();
  }
}
void joint_callback(const sensor_msgs::JointState& msg){

  msg_from_joints=msg;
}
bool callback_modality(ur3_control::aruco_service::Request &req, ur3_control::aruco_service::Response &res){


  if(show_log)
     ROS_INFO("Bridge:Chiamata ricevuta:\nModalita:%s\n\n",req.modality.c_str());

  if(str_md_bpa==req.modality){
    bool_md_bpa=true;
  }
  if(str_md_stop_bpa==req.modality){
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
int main(int argc, char** argv){

  signal(SIGINT, signal_callback_handler);
  ros::init(argc, argv, "bridge");


  static ros::AsyncSpinner spinner(1);

  ros::Subscriber sub,sub_joint;
  ros::NodeHandle n;
  ros::ServiceServer serv;
  pub_traj_cancel = n.advertise<actionlib_msgs::GoalID>("/arm_controller/follow_joint_trajectory/cancel", 100);
  sub=n.subscribe("/aruco_bridge_opencv",1,cv_callback);
  sub_joint=n.subscribe("/joint_states",1,joint_callback);
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
