//Piacere, Federico
#include "library.h"


void MenuDiSceltaGiunti();
void MenudDiSceltaEndEffectorAssi();
void MenudDiSceltaEndEffectorTarget();
void MenudDiSceltaEndEffector();
void MenuPosizioniBase();
void MenuMovimentiBase();
void MenuDiScelta();
void joystick();
void Start();
void Controller();
void Interazione_Environment();
void Menu_Di_Scelta_Prove();
void MenuDiSceltaOpzioni();

bool msg_to_be_processed=false;
ur3_control::UserInterface::Request msg_from_interface;
bool callback_modality(ur3_control::UserInterface::Request &req, ur3_control::UserInterface::Response &res){
  msg_from_interface=req;
  msg_to_be_processed=true;
  return true;
}
void esegui_msg_from_inteface(){
  bool_exit=false;
  if(msg_to_be_processed){
      ROS_INFO("Chiamata ricevuta:\nModalita:%s\n\n",msg_from_interface.modality.c_str());
      if(msg_from_interface.modality=="pose"){
          move_to_pose(msg_from_interface.target_pose,true);
      }
      if(msg_from_interface.modality=="pose_randomOrientation"){
          move_to_pose(msg_from_interface.target_pose,false);
      }
      if(msg_from_interface.modality=="joints"){
          vector<double> joint_group_positions=robot->getCurrentJointValues();
          joint_group_positions[0]=grad_to_rad(msg_from_interface.target_joints[0]);
          joint_group_positions[1]=grad_to_rad(msg_from_interface.target_joints[1]);
          joint_group_positions[2]=grad_to_rad(msg_from_interface.target_joints[2]);
          joint_group_positions[3]=grad_to_rad(msg_from_interface.target_joints[3]);
          joint_group_positions[4]=grad_to_rad(msg_from_interface.target_joints[4]);
          joint_group_positions[5]=grad_to_rad(msg_from_interface.target_joints[5]);
          robot->setJointValueTarget(joint_group_positions);
          success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
          ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
          robot->move();

      }
      if(msg_from_interface.modality=="save_aruco_in_txt"){
        save_aruco_in_txt();
      }
      if(msg_from_interface.modality=="exit"){
          exit_from_all();
      }
      if(msg_from_interface.modality=="joystick_Joints"){
          vector<double> joint_group_positions=robot->getCurrentJointValues();
          joint_group_positions[0]=joint_group_positions[0]+grad_to_rad(msg_from_interface.target_joints[0]);
          joint_group_positions[1]=joint_group_positions[1]+grad_to_rad(msg_from_interface.target_joints[1]);
          joint_group_positions[2]=joint_group_positions[2]+grad_to_rad(msg_from_interface.target_joints[2]);
          joint_group_positions[3]=joint_group_positions[3]+grad_to_rad(msg_from_interface.target_joints[3]);
          joint_group_positions[4]=joint_group_positions[4]+grad_to_rad(msg_from_interface.target_joints[4]);
          joint_group_positions[5]=joint_group_positions[5]+grad_to_rad(msg_from_interface.target_joints[5]);
          robot->setJointValueTarget(joint_group_positions);
          success = (robot->plan(my_plan) == MoveItErrorCode::SUCCESS);
          ROS_INFO_NAMED("tutorial", "%s", success ? "SUCCESS" : "FAILED");
          robot->move();
          stampa_giunti();
      }
      if(msg_from_interface.modality=="automazione_pannello_posizioneCorretta"){
          PosizioniBase(str_pannello);
      }
      if(msg_from_interface.modality=="automazione_pannello_Completa"){
      }
      if(msg_from_interface.modality=="automazione_pannello_MoveToSelectedAruco"){
        ur3_control::aruco_serviceResponse msg_from_bridge=bridge_service(str_md_rd,"");

          if(msg_from_bridge.id_aruco==1 || msg_from_bridge.id_aruco==2 || msg_from_bridge.id_aruco==3 ||
             msg_from_bridge.id_aruco==4 || msg_from_bridge.id_aruco==5 || msg_from_bridge.id_aruco==6 ||
             msg_from_bridge.id_aruco==7 || msg_from_bridge.id_aruco==8 || msg_from_bridge.id_aruco==9 ){
            action_aruco_button(to_string(msg_from_bridge.id_aruco));
          }
          else if(msg_from_bridge.id_aruco==13){
            right_panel();
          }
          else if(msg_from_bridge.id_aruco==10){
            left_panel();
          }
          else{
            Affine_valid T_aruco_valid=homo_0_aruco_elaration();
            if(T_aruco_valid.valid){
              pre_grasp(T_aruco_valid.homo_matrix);
            }
          }




      }
      if(msg_from_interface.modality=="automazione_pannello_nextAruco"){
      //bridge_service(str_md_next_aruco,msg_from_interface.second_information);
      cambia_aruco(msg_from_interface.second_information);
  }
      if(msg_from_interface.modality=="ruota_e_cerca_aruco"){

        ruota_e_cerca_aruco();
        usleep(300000);//0.3 sec
        move_aruco_to_center_of_camera(0);
      }
      if(msg_from_interface.modality=="ruota_e_cerca_pannello"){

        ruota_e_cerca_pannello();
        usleep(300000);//0.3 sec
      }
      if(msg_from_interface.modality=="centra_aruco"){
        int id;
        int zoom;
        cout<<"Quale aruco?";
        cin>>id;
        centra_aruco_nella_camera(id,0);
      }
      if(msg_from_interface.modality=="centra_aruco_and_zoom_in"){
        int id;
        cout<<"Quale aruco?";
        cin>>id;
        centra_aruco_nella_camera(id,20);
      }
      if(msg_from_interface.modality=="centra_aruco_and_zoom_out"){
        int id;
        cout<<"Quale aruco?";
        cin>>id;
        centra_aruco_nella_camera(id,-20);
      }
      if(msg_from_interface.modality=="automazione_go_pos_iniziale"){

        PosizioniBase(str_pos_iniziale);

      }
      if(msg_from_interface.modality=="automazione_debug1"){
        ROS_INFO("Debug1");
        load_parameters();
      }
      if(msg_from_interface.modality=="automazione_debug2"){

        ROS_INFO("Debug2");

      }
      if(msg_from_interface.modality=="controlla_gripper"){
        action_gripper(msg_from_interface.second_information);
      }
      if(msg_from_interface.modality=="joystick"){
        robot->setPlanningTime(0);
        controlla(msg_from_interface.second_information[0]);
        robot->setPlanningTime(4);
      }
      if(msg_from_interface.modality=="Stampa_pose_robot"){
        stampa_Pose(robot->getCurrentPose().pose);
        stampa_giunti();
      }
      if(msg_from_interface.modality=="salva_aruco"){
        int id;
        cout<<"Quale aruco vuoi salvare?";
        cin>>id;
        aggiorna_aruco_selezionato(id);

      }
      if(msg_from_interface.modality=="esplora_tutto"){
        esplora_tutti_gli_aruco();
      }
      if(msg_from_interface.modality=="superdebug"){
        prova();
      }
      msg_to_be_processed=false;
  }

}
void param_control(){

  int current_objective=0;

  ros::NodeHandle start_node;
  start_node.getParam("objective",current_objective);

  switch(current_objective){
  case 1:{

    esplora_tutti_gli_aruco();
    stampa_aruco_su_excel();
    PosizioniBase(str_pos_iniziale);
    action_gripper("open");


    start_node.setParam("objective",0);
    break;}
  case 2:{
    // button sequence
    ROS_INFO("Aruco buttons:");
    string buttons_string;
    //get button sequence
    start_node.getParam("buttons_sequence",buttons_string);
    //remove spaces, split elements

    string button1,button2,button3,button4;
    button1="?";button2="?";button3="?";button4="?";
    if(buttons_string.size()==1){
      button1=buttons_string[0];
      ROS_INFO("Sto andando a premere 1 buttone");
    }
    if(buttons_string.size()==3){
      button1=buttons_string[0];
      button2=buttons_string[2];
      ROS_INFO("Sto andando a premere 2 buttoni");
    }
    if(buttons_string.size()==5){
      button1=buttons_string[0];
      button2=buttons_string[2];
      button3=buttons_string[4];
      ROS_INFO("Sto andando a premere 3 buttoni");
    }
    if(buttons_string.size()==7 || buttons_string.size()==8){
      button1=buttons_string[0];
      button2=buttons_string[2];
      button3=buttons_string[4];
      button4=buttons_string[6];
      ROS_INFO("Sto andando a premere 4 buttoni");
    }
    cout<<"Button_string:"<<buttons_string<<endl;
    cout<<"Aruco id:"<<button1<<button2<<button3<<button4<<endl;

    string button_selected;
    button_selected=button1;
    if(button_selected>="0" && button_selected<="9"){

    cambia_aruco(button_selected);
    action_aruco_button(button_selected);
    sleep(1);

    }
    else{
      ROS_INFO("First button skipped, charapter not correct");
    }
    button_selected=button2;
    if(button_selected>="0" && button_selected<="9"){

    cambia_aruco(button_selected);
    action_aruco_button(button_selected);
    sleep(1);

    }
    else{
      ROS_INFO("Second button skipped, charapter not correct");
    }
    button_selected=button3;
    if(button_selected>="0" && button_selected<="9"){

    cambia_aruco(button_selected);
    action_aruco_button(button_selected);
    sleep(1);

    }
    else{
      ROS_INFO("Third button skipped, charapter not correct");
    }
    button_selected=button4;
    if(button_selected>="0" && button_selected<="9"){

    cambia_aruco(button_selected);
    action_aruco_button(button_selected);
    sleep(1);

    }
    else{
      ROS_INFO("Fourth button skipped, charapter not correct");
    }
    start_node.deleteParam("button_sequence");
    start_node.setParam("objective",0);
    break;}
  case 3:{

    solleva_imu();

    start_node.setParam("objective",0);
    break;}
  case 4:{
    // place imu
    double imu_angle;
    // get imu angle
    start_node.getParam("imu_angle",imu_angle);
    //pass imu angle as orientan, degrees
    ROS_INFO_STREAM( "imu angle  " << imu_angle);


    go_and_attach_imu(imu_angle);

    start_node.deleteParam("imu_angle");
    start_node.setParam("objective",0);
    sleep(4);
    break;}
  case 5:{
    // open inspection window
    if(Aruco_values[ID_INSPECTION_WINDOW_COVER].valid){

      flagRightBoxCreated=false;
       se_aruco_individuato_add_collision(ID_INSPECTION_WINDOW_COVER);
    }
    solleva_coperchio();

    current_objective=0;
    start_node.setParam("objective",0);
    break;
  }
  case 6:{
    // inspect window
    go_and_recognize_id_in_inspection_window();
    start_node.setParam("objective",0);
    break;
  }
  case 7:{
    // push button of hidden id
    string id_nascosto;
    start_node.getParam("hidden_id",id_nascosto);

    // pass hidded_id as id of aruco to press

    ROS_INFO_STREAM( "hidden id " << id_nascosto);

    action_aruco_button(id_nascosto);

    start_node.setParam("objective",0);
    break;
  }
  case 8:{
    // place cover back

    start_node.setParam("objective",0);
    break;
  }
  case 9:{
    PosizioniBase(str_pos_iniziale);
    action_gripper("open");


    start_node.setParam("objective",0);
    break;
  }
  case 42:{
    exit_from_all();

    start_node.deleteParam("objective");
    break;
  }
  default:{
    //wait
    break;
  }
  }
}

int main(int argc, char** argv)
{
  // Initialize the ROS Node "roscpp_hello_world"

  signal(SIGINT, signal_callback_handler);
  ros::init(argc, argv, "simulation");
  static ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(1.0).sleep();
  ros::NodeHandle node_handle,node_service_aruco;
  using namespace std;


     MoveGroupInterface move_group(PLANNING_GROUP);


  robot=&move_group;
  //namespace rvt = rviz_visual_tools;
  //moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  //visual_tools.deleteAllMarkers();

  gazebo_model_state_pub = node_handle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 100);
  pose_object_client = node_handle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  ros::ServiceServer serv=node_handle.advertiseService("/user_interface_serv", callback_modality);
  pub_gripper = node_handle.advertise<std_msgs::String>("/gripper_command", 1);
  pub_traj_cancel = node_handle.advertise<actionlib_msgs::GoalID>("/move_group/cancel", 1);

  //INIZIALIZZAZIONI
  ALL_INITIAL_VOIDS();


  string scelta_tipologia;
  node_handle.getParam("interface",scelta_tipologia);


//  do{
//      cout<<"0)Esci"<<endl<<"1) Launcher menu"<<endl<<"2) User_interface menu"<<endl<<"3) Old menu"<<endl<<"Choice:";
//      cin>>scelta_tipologia;
//  }while(!bool_exit && scelta_tipologia!="0" && scelta_tipologia!="1" && scelta_tipologia!="2" && scelta_tipologia!="3");

  if(scelta_tipologia[0]=='1'){

    while(ros::ok() && !bool_exit){
      param_control();
    }

  }
  else if(scelta_tipologia[0]=='2'){

    while(ros::ok() && !bool_exit){
      ros::spinOnce();
      esegui_msg_from_inteface();
    }

  }
  else if(scelta_tipologia[0]=='3'){

    Start();

  }
  else if(scelta_tipologia[0]=='0'){
    bridge_service("exit","");
    bool_exit=true;
  }


  return 0;
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
  double x=0.3,y=0.3,z=0.3;
  bool void_sdr_solidale=false;
  Vector3d translation(0,0,0);
  do{
    VettoreSomma = target.position.x*target.position.x + target.position.y*target.position.y + (target.position.z-0.1)*(target.position.z-0.1);
    if(VettoreSomma<0.5*0.5) Errore="False";
    else Errore="True";
    cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
    cout<<endl<<"0)Esci"<<endl<<"1)Orientamento casuale:"<<(Orientamento ? "ATTIVO" : "NON ATTIVO")<<endl<<"2)X:"<<x<<endl<<"3)Y:"<<y<<endl<<"4)Z:"<<z<<endl<<"5)Sdr Solidale:"<<(void_sdr_solidale?"YES":"NO")<<endl<<"6)Muovi"<<endl<<"Errore:"<<Errore<<endl<<"RPY:"<<alpha<<" "<<beta<<" "<<gamma<<endl<<"Scelta:";
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
        cin>>x;
        break;

      }
      case 3:{

        cout<<endl<<endl<<endl;
        cout<<"Inserisci Y:";
        cin>>y;
        break;

      }
      case 4:{

        cout<<endl<<endl<<endl;
        cout<<"Inserisci Z:";
        cin>>z;
        break;

      }
      case 5:{
      void_sdr_solidale=!void_sdr_solidale;
      break;
      }
      case 6:{
      if(!void_sdr_solidale){
        target.position.x=x;
        target.position.y=y;
        target.position.z=z;
        move_to_pose(target,!Orientamento);//Orientamento
      }else{
        Vector3d temp(x,y,z);
        translation=temp;

        Pose target=pose_traslation_solidale(translation);

        move_to_pose(target,true);
      }
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
  string comando;
  cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
  cout<<endl<<"Inserisci una stringa tra queste elencate:";
  cout<<endl<<str_r<<endl<<str_ta<<endl<<str_tb<<endl<<str_up<<endl<<str_home<<endl<<str_centrifuga<<endl<<str_pannello<<endl;
  cout<<"Scelta:";
  cin>>comando;
  PosizioniBase(comando);
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
          sleep(1.0);
          robot->execute(my_plan);
          break;
      }
      case 2:{
        PosizioniBase(str_tb);
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
void joystick(){
  input_char='0';
  ROS_INFO("\n\nPremi h, per info.Da qui in poi inserisci i comandi:");
  joystick_ready=false;
  while (input_char!='e')
  {
    system("stty raw");
    input_char=getch();
    system("stty cooked");
    joystick_ready=true;

    if(input_char=='h'){
      publish_joystick_info();
    }

  }

}
void Controller(){
  ROS_INFO("\nModalita joystick! Attendo comando dal nodo joystick");
  joystick_ready=false;
  input_char='0';
  boost::thread thread_controller(joystick);
  do{
    if(joystick_ready==true){
      controlla(input_char);
      joystick_ready=false;
    }
    }while(input_char!='e');

}
void Interazione_Environment(){

  unsigned int comando;
  cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
  cout<<endl<<"0)Esci"<<endl<<"1)Movimento automatizzato gara"<<endl<<"2)Muoviti fino alla centrifuga"<<endl<<"21)Muoviti quasi vicino alla centrifuga"<<endl<<"3)pick object"<<endl<<"4)unpick object"<<endl<<"5)move to take object"<<endl<<"6)Individua_aruco"<<endl<<"7)Individua e prendi aruco"<<endl<<"8)Pannello"<<endl<<"Scelta:";
  cin>>comando;
  cout<<endl<<endl<<endl;
  switch(comando){
    case 1:{

      automatizzato();
      break;

  }
    case 2:{
    move_to_centrifuga();
      break;

    }
    case 21:{
      move_near_to_centrifuga();
      break;
  }
    case 3:{
      cout<<endl<<"nome oggetto: ";
      cin>>nome_oggetto;
      boost::thread pick_thread(pick, nome_oggetto,0.191);
      break;

    }
    case 4:{
      picked=false;
      break;

    }
    case 5:{
    take_object();
    break;
    }
    case 6:{
      Pose aruco_pose_solidale;
      individua_aruco(&aruco_pose_solidale);
      break;
    }
    case 7:{
  move_to_aruco();
  break;
}
    case 8:{
        aruco_pannello();
        break;
    }
    }
}
void Menu_Di_Scelta_Prove(){
  unsigned int comando;
  do{
      cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
      cout<<endl<<"0)Esci"<<endl<<"1)Movimento 360"<<endl<<"Scelta:";
      cin>>comando;
      cout<<endl<<endl<<endl;
      switch(comando){
        case 1:{

           ros:: Publisher pub_traj_cancel;
           ros::NodeHandle node_traj_cancel;
           boost::thread ruota_thread(ruota_360);

           pub_traj_cancel = node_traj_cancel.advertise<actionlib_msgs::GoalID>("/execute_trajectory/cancel", 100);
           sleep(10);

           ROS_INFO("Stopping trajectory");
           actionlib_msgs::GoalID msg_traj_cancel;
           pub_traj_cancel.publish(msg_traj_cancel);
          break;

      }
        case 2:{
          break;

        }
        case 3:{
          break;

        }
        case 4:{

          break;

        }
      }
  }while(comando!=0);
}
void MenuDiSceltaOpzioni(){
  unsigned int comando;
  do{
      cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
      cout<<endl<<"0)Esci"<<endl<<"1)Stampa valori robot"<<"\nScelta:";
      cin>>comando;
      cout<<endl<<endl<<endl;
      switch(comando){
        case 1:{

          stampa_giunti();
          stampa_Pose(robot->getCurrentPose().pose);


          break;

      }
        case 2:{

          break;
        }
        case 3:{
          break;

        }
        case 4:{

          break;

        }
      }
  }while(comando!=0);
}
void Start(){
  int comando;
  do{
      cout<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl<<endl;
      cout<<endl<<"0)Esci"<<endl<<"1)Menu Di Scelta"<<endl<<"2)Joystick"<<endl<<"3)Interagisci con Environment"<<endl<<"4)Menu Prove"<<endl<<"5)Opzioni"<<endl<<"Scelta:";
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
        case 3:{
          Interazione_Environment();
          break;

        }
        case 4:{

          Menu_Di_Scelta_Prove();
          break;

        }
        case 5:{
        MenuDiSceltaOpzioni();
        break;
      }
      }
  }while(comando!=0);
}
/*
ROBA DA FARE:
  - Aggiustare parametri pianificazione
*/
