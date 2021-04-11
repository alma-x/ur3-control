#include "ros/ros.h"
#include "std_msgs/String.h"

#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <tf/tf.h>
#include <sstream>
#include<iostream>
using namespace std;
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
int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("almax_joystick", 1);
  ros::Rate loop_rate(10);
  char input_char='0';
  ROS_INFO("\n\nPremi h, per info.Da qui in poi inserisci i comandi:");
  while (input_char!='e')
  {
    system("stty raw");
    input_char=getch();
    system("stty cooked");

    std_msgs::String msg;
    std::stringstream ss;

    ss<<input_char;
    msg.data = ss.str();
    chatter_pub.publish(msg);
    if(input_char=='h'){
      publish_joystick_info();
    }
    ros::spinOnce();
  }

  return 0;
}
