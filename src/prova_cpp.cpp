#include "ur3_control/aruco_service.h"
#include "ros/ros.h"
#include "ros/service.h"
#include "iostream"
#include "stdio.h"
using namespace std;
int main(int argc, char** argv){
  ros::init(argc, argv, "simulation");
  ros::NodeHandle node_handle;

  ros::ServiceClient client1;
  client1 = node_handle.serviceClient<ur3_control::aruco_service>("/add_two_ints");
  ur3_control::aruco_service x;
  client1.call(x);
  printf("%f",x.response.x);
}
