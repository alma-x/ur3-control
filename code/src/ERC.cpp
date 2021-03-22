#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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
PlanningSceneInterface *interface;
vector<CollisionObject> collision_objects;
string CollisionObjectID;
vector<string> object_ids;


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
void add_block(){
//  cout<<"Inserisci alpha:";
//  cin>>alpha;
//  cout<<"Inserisci beta:";
//  cin>>beta;
  CollisionObject collision_object;
  tf2::Quaternion quat;
  shape_msgs::SolidPrimitive primitive;
  Pose box_pose;
  collision_object.header.frame_id = robot->getPlanningFrame();
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.05;

  box_pose.position.x = 1;
  box_pose.position.y = 1;
  box_pose.position.z = 1;
  SetPoseOrientationRPY(&box_pose,0,0,45);
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  collision_objects.push_back(collision_object);
  interface->addCollisionObjects(collision_objects);
  CollisionObjectID=collision_object.id;
  object_ids.push_back(CollisionObjectID);
}
void remove_block(){
  //collision_objects.pop_back();
  collision_objects.clear();
  interface->removeCollisionObjects(object_ids);
  object_ids.clear();
}
void controlla(int key_int){
    string k;
    Pose targ=robot->getCurrentPose().pose;
    char key=key_int;
    unsigned int giunto=0;
    int angolo_base=30;
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
void MenuDiSceltaGiunti(){
  unsigned int comando,GiuntoScelto=1;
  int AngoloIncrementato=15;
  string PositivitaRotazione="positivo";
  do{
    cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
    cout<<endl<<"0)Esci"<<endl<<"1)Cambia senso di rotazione, per ora:"<<PositivitaRotazione<<endl<<"2)Cambia angolo, per ora:"<<AngoloIncrementato<<endl<<"3)Cambia giunto da muovere, per ora:"<<GiuntoScelto<<endl<<"4)Muovi"<<endl<<"Scelta:";
    cin>>comando;
    switch(comando){
      case 1:{
        if(PositivitaRotazione=="positivo") PositivitaRotazione="negativo";
        else PositivitaRotazione="positivo";

        break;
      }
      case 2:{

        cout<<endl<<endl<<endl;
        cout<<"Inserisci angolo in grad:";
        cin>>AngoloIncrementato;

        break;
      }
      case 3:{

        cout<<endl<<endl<<endl;
        cout<<"Inserisci giunto da muovere:";
        cin>>GiuntoScelto;

        break;
      }
      case 4:{
        if(PositivitaRotazione=="positivo")
          ruotagiunto(GiuntoScelto,AngoloIncrementato);

        if(PositivitaRotazione=="negativo")
          ruotagiunto(GiuntoScelto,-AngoloIncrementato);

        break;
      }
    }
  }while(comando!=0);
}
void MenudDiSceltaEndEffectorAssi(){
  unsigned int comando;
  double Spostamento=0.02,VettoreSomma;
  bool Orientamento=true, NewSDR=false;
  int AngleSDR=0;
  string AsseScelto="x",PositivitaSpostamento="positivo",Errore;
  do{
    Pose target=robot->getCurrentPose().pose;
    cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
    cout<<endl<<"0)Esci"<<endl<<"1)Cambia verso di spostamento, per ora:"<<PositivitaSpostamento<<endl<<"2)Cambia distanza di spostamento, per ora:"<<Spostamento<<endl<<"3)Cambia asse da muovere, per ora:"<<AsseScelto<<endl<<"4)Vuoi mantenere lo stesso Orientamento:"<<Orientamento<<endl<<"5)Vuoi il SDR ruotato?:"<<(NewSDR ? "Si" : "NO")<<endl<<"6)Muovi"<<endl<<"Scelta:";
    cin>>comando;
    switch(comando){
      case 1:{
        if(PositivitaSpostamento=="positivo") PositivitaSpostamento="negativo";
        else PositivitaSpostamento="positivo";

        break;
      }
      case 2:{

        cout<<endl<<endl<<endl;
        cout<<"Inserisci distanza da percorrere in metri:";
        cin>>Spostamento;
        break;

      }
      case 3:{
        if(AsseScelto=="x")
          AsseScelto="y";
        else
          if(AsseScelto=="y")
            AsseScelto="z";
          else
            if(AsseScelto=="z")
              AsseScelto="x";
        break;

      }
      case 4:{

        Orientamento=!Orientamento;
        break;

      }
      case 5:{
      NewSDR=!NewSDR;
          if(NewSDR){
            AngleSDR=30;
          }
          if(!NewSDR){
            AngleSDR=0;
          }
      break;
      }
      case 6:{
        if(PositivitaSpostamento=="positivo"){
          if(AsseScelto=="x") {
            target.position.x+=Spostamento*cos(grad_to_rad(AngleSDR));
            target.position.y+=Spostamento*sin(grad_to_rad(AngleSDR));
          }
          if(AsseScelto=="y") {

            target.position.x-=Spostamento*sin(grad_to_rad(AngleSDR));
            target.position.y+=Spostamento*cos(grad_to_rad(AngleSDR));
          }
          if(AsseScelto=="z") target.position.z+=Spostamento;
        }
        if(PositivitaSpostamento=="negativo"){
          if(AsseScelto=="x") {
            target.position.x-=Spostamento*cos(grad_to_rad(AngleSDR));
            target.position.y-=Spostamento*sin(grad_to_rad(AngleSDR));
          }
          if(AsseScelto=="y") {

            target.position.x+=Spostamento*sin(grad_to_rad(AngleSDR));
            target.position.y-=Spostamento*cos(grad_to_rad(AngleSDR));
          }
          if(AsseScelto=="z") target.position.z-=Spostamento;
        }
        VettoreSomma = target.position.x*target.position.x + target.position.y*target.position.y + (target.position.z-0.1)*(target.position.z-0.1);
        if(VettoreSomma>0.5*0.5)
          cout<<"Secondo me muori";//Poi si aggiungerà in qualche modo, cosi non è molto utile
        move_to_pose(target,Orientamento);

        break;
      }
    }
  }while(comando!=0);
}
void MenudDiSceltaEndEffectorTarget(){
  unsigned int comando;
  bool Orientamento=true;
  double VettoreSomma;
  string Errore;
  Pose target;
  int alpha=0,beta=0,gamma=0;
  target.position.x=0.3;
  target.position.y=0.3;
  target.position.z=0.3;
  do{
    VettoreSomma = target.position.x*target.position.x + target.position.y*target.position.y + (target.position.z-0.1)*(target.position.z-0.1);
    if(VettoreSomma<0.5*0.5) Errore="False";
    else Errore="True";
    cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
    cout<<endl<<"0)Esci"<<endl<<"1)Orientamento casuale:"<<(Orientamento ? "ATTIVO" : "NON ATTIVO")<<endl<<"2)X:"<<target.position.x<<endl<<"3)Y:"<<target.position.y<<endl<<"4)Z:"<<target.position.z<<endl<<"5)Muovi"<<endl<<"Errore:"<<Errore<<endl<<"RPY:"<<alpha<<" "<<beta<<" "<<gamma<<endl<<"Scelta:";
    cin>>comando;
    switch(comando){
      case 1:{

        Orientamento=!Orientamento;

        if(!Orientamento){
          tf2::Quaternion quat;
          cout<<"Inserisci alpha:";
          cin>>alpha;
          cout<<"Inserisci beta:";
          cin>>beta;
          cout<<"Inserisci gamma:";
          cin>>gamma;
          quat.setRPY(grad_to_rad(alpha),grad_to_rad(beta),grad_to_rad(gamma));
          tf2quat_to_pose(quat,&target);
}
        break;

      }
      case 2:{

        cout<<endl<<endl<<endl;
        cout<<"Inserisci X:";
        cin>>target.position.x;
        break;

      }
      case 3:{

        cout<<endl<<endl<<endl;
        cout<<"Inserisci Y:";
        cin>>target.position.y;
        break;

      }
      case 4:{

        cout<<endl<<endl<<endl;
        cout<<"Inserisci Z:";
        cin>>target.position.z;
        break;

      }
      case 5:{
      move_to_pose(target,!Orientamento);//Orientamento
      break;

      }

    }
  }while(comando!=0);
}
void MenudDiSceltaEndEffector(){
  unsigned int comando;
  do{
    cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
    cout<<endl<<"0)Esci"<<endl<<"1)Muovi lungo gli assi"<<endl<<"2)Scegli target"<<endl<<"Scelta:";
    cin>>comando;
    switch(comando){
      case 1:{

        MenudDiSceltaEndEffectorAssi();
        break;

      }
      case 2:{

        MenudDiSceltaEndEffectorTarget();
        break;

      }

    }
  }while(comando!=0);
}
void MenuPosizioniBase(){
  unsigned int comando;
  do{
    cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
    cout<<endl<<"0)Esci"<<endl<<"1)PROVAAA"<<endl<<"2)Full Stand"<<endl<<"3)Testa in basso"<<endl<<"4)Testa in alto"<<endl<<"Scelta:";
    cin>>comando;
    switch(comando){
    case 1:{

      PosizioniBase(1);
      /*



      const robot_state::JointModelGroup* joint_model_group =robot->getCurrentState()->getJointModelGroup(PLANNING_GROUP);


      geometry_msgs::Pose target_pose1;
      SetPoseOrientationRPY(&target_pose1,0,0,0);
      target_pose1.position.x = 0.3;
      target_pose1.position.y = -0.3;
      target_pose1.position.z = 0.3;
      //robot->setPoseTarget(target_pose1);

      moveit_msgs::OrientationConstraint ocm;
      ocm.link_name = "wrist_3_link";

      ocm.header.frame_id = "base_link";
      tf2::Quaternion quat;
      quat.setRPY(grad_to_rad(0),grad_to_rad(0),grad_to_rad(0));
      ocm.orientation.x=quat.getX();
      ocm.orientation.y=quat.getY();
      ocm.orientation.z=quat.getZ();
      ocm.orientation.w=quat.getW();

      ocm.absolute_x_axis_tolerance = 0.2;
      ocm.absolute_y_axis_tolerance = 0.2;
      ocm.absolute_z_axis_tolerance = 0.2;
      ocm.weight = 1.0;

      moveit_msgs::Constraints test_constraints;
      test_constraints.orientation_constraints.push_back(ocm);
      robot->setPathConstraints(test_constraints);


      robot->setPoseTarget(target_pose1);

      robot->setPlanningTime(10.0);

      success = (robot->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");
      robot->execute(my_plan);
      robot->clearPathConstraints();*/
      break;

    }
    case 2:{

      PosizioniBase(2);
      break;

    }
    case 3:{

      PosizioniBase(3);
      break;

    }case 4:{

      PosizioniBase(4);
      break;

    }

    }
  }while(comando!=0);
}
void MenuMovimentiBase(){
  unsigned int comando;
  do{
    cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
    cout<<endl<<"0)Esci"<<endl<<"1)Rotazione attorno ad un centro"<<endl<<"2)Prendi spina dal basso"<<endl<<"Scelta:";
    cin>>comando;
    switch(comando){
      case 1:{
      vector<Pose> waypoints;
      Pose ee_point_goal;
      double r,resolution,step,xc,yc,zc;

      r=0.1;
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
          sleep(5.0);
          robot->execute(my_plan);
          break;

      }
      case 2:{
        PosizioniBase(3);
        sleep(2.0);

        break;

      }

    }
  }while(comando!=0);
}
void MenuDiScelta(){


  unsigned int comando;
  do{
      cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
      cout<<endl<<"0)Esci"<<endl<<"1)Muovi giunto"<<endl<<"2)Muovi end effector"<<endl<<"3)Posizioni base"<<endl<<"4)Movimementi base"<<endl<<"5)Cambia Parametri"<<endl<<"Scelta:";
      cin>>comando;
      cout<<endl<<endl<<endl;
      switch(comando){
        case 1:{

          MenuDiSceltaGiunti();
          break;

      }
        case 2:{

          MenudDiSceltaEndEffector();
          break;

        }
        case 3:{

          MenuPosizioniBase();
          break;

        }
        case 4:{

        MenuMovimentiBase();
        break;

        }
        case 5:{
        string rangeparam;
        cout<<"Inserisci Range:";
        cin>>rangeparam;
        map<string,string> m;
        m.insert(pair<string, string>("range", rangeparam));
        robot->setPlannerParams("RRTConnectkConfigDefault", PLANNING_GROUP, m);
        m.clear();
        break;

        }
      }
  }while(comando!=0);

}
void Controller(){
  int input;
  do{

    cout<<endl<<endl<<"Press any key to continue... (h for info)"<<endl;

    system("stty raw");
    input=getch();
    system("stty cooked");
    controlla(input);

  }while(input!='e');
}
void Start(){
  unsigned int comando;
  do{
      cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
      cout<<endl<<"0)Esci"<<endl<<"1)Menu Di Scelta"<<endl<<"2)Controller"<<endl<<"Scelta:";
      cin>>comando;
      cout<<endl<<endl<<endl;
      switch(comando){
        case 1:{

          robot->setPlanningTime(2);
          robot->setNumPlanningAttempts(1000);
          MenuDiScelta();
          break;

      }
        case 2:{
          robot->setPlanningTime(0);
          robot->setNumPlanningAttempts(0);
          Controller();
          break;

        }
      }
  }while(comando!=0);
}
int main(int argc, char** argv)
{
  // Initialize the ROS Node "roscpp_hello_world"
  ros::init(argc, argv, "simulation");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(1.0).sleep();

  MoveGroupInterface move_group(PLANNING_GROUP);
  PlanningSceneInterface planning_scene_interface;
  robot=&move_group;
  robot->setPlannerId("RRTConnectkConfigDefault");
  //robot->setPlannerId("RRTstarkConfigDefault");
  //robot->setPlanningTime(10);
  interface=&planning_scene_interface;

  Start();
  ros::shutdown();
  return 0;

}

