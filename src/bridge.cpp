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
ros::ServiceClient client;
bool bool_md_bpa=false;
bool bool_exit=false;
void stampa_cv_msg(const ur3_control::cv_to_bridge msg){
ROS_INFO("Message received:\n x:%f \n y:%f \n z:%f \n success:%s",msg.x,msg.y,msg.z,(msg.success)? "success":"not success");
}
void cv_callback(const ur3_control::cv_to_bridge& msg){

  msg_from_cv=msg;


  if(bool_md_bpa && msg_from_cv.success){

    bool_md_bpa=false;

    ros::NodeHandle n_stop;
    ros::Publisher pub_traj_cancel = n_stop.advertise<actionlib_msgs::GoalID>("/execute_trajectory/cancel", 100);
    actionlib_msgs::GoalID msg_traj_cancel;
    msg_traj_cancel.id="s";
    pub_traj_cancel.publish(msg_traj_cancel);
    ROS_INFO("Aruco trovato, traiettoria cancellata");
    robot->stop();
  }
}
bool callback_modality(ur3_control::aruco_service::Request &req, ur3_control::aruco_service::Response &res){


  ROS_INFO("Chiamata ricevuta:\nModalita:%s\n\n",req.modality.c_str());

  if(str_md_bpa==req.modality){
    bool_md_bpa=true;
  }
  if(str_md_stop_bpa==req.modality){
    bool_md_bpa=false;
  }
  if(str_md_next_aruco==req.modality){

      ur3_control::cv_server cv_service_msg;
      cv_service_msg.request.next_aruco=true;
      client.call(cv_service_msg);

      res.moreTargets=cv_service_msg.response.moreTargets;
      ROS_INFO("targets remaining:%d",res.moreTargets);
  }
  if("exit"==req.modality){
      bool_exit=true;
      ur3_control::cv_server cv_service_msg;
      cv_service_msg.request.message="exit";
      client.call(cv_service_msg);
  }
  res.aruco_found=msg_from_cv.success;
  res.x=msg_from_cv.x;
  res.y=msg_from_cv.y;
  res.z=msg_from_cv.z;
  res.vector=msg_from_cv.vector;

  return true;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "bridge");


  static ros::AsyncSpinner spinner(1);

  ros::Subscriber sub;
  ros::NodeHandle n;
  ros::ServiceServer serv;
  sub=n.subscribe("/aruco_bridge_opencv",1,cv_callback);
  serv=n.advertiseService("aruco_modality", callback_modality);
  client = n.serviceClient<ur3_control::cv_server>("/cv_server");
  MoveGroupInterface move_group(PLANNING_GROUP);

  robot=&move_group;
  msg_from_cv.success=false;
  bool_md_bpa=false;
  while(ros::ok && !bool_exit){
  ros::spinOnce();
  }
  return 0;
}
