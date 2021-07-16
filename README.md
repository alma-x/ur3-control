# ur3-control package
https://github.com/PUT-UGV-Team/ur3_erc_docker

# RUN CODE GARA
1) Devi avere tutte e tre le repo scaricate: almasim, ur3-control, simulation-cv
2) roslaunch simulation simulator_small.launch      oppure      roslaunch simulation simulator.launch
3) roslaunch ur3_control launcher_gara.launch
4) rosrun ur3_control UI_gara.py

# ALTRO
Questo è il catkin_pkg da scaricare dentro workspace/src

Come minimizzare il codice in Qt(unfold all):
Tools->Options->Enviroment->Keyboard
Nel campo filter: cerca "UnFoldAll"
Nel campo shortcut inserire con quale shortcut si vuole chiamare l'azione. "Ctrl+ì"
Apply


Come utilizzare joystick all interno del pacchetto code:
1)
Controllare che il nodo joystick sia presente nel cmakelist del pacchetto.
2)
roslaunch ur3_moveit_config demo.launch
rosrun code ERC
rosrun code joystick
3)
All'interno di ERC selezionare modalità joystick. A quel punto dal processo joystick inserire i comandi


Come usare empty_world_server.launch (per avere solo il gazebo server e non il client):

1) esegui su terminale:  roscd gazebo_ros

2) entra nella cartella launch : cd launch

3) incolla empty_world_server.launch

4) vai su (partendo dalla catkin_ws): catkin_ws->src->ERC_2021_simulator->ur_gazebo->launch->ur3.launch

5) commenta la riga  <include file="$(find gazebo_ros)/launch/empty_world.launch"> e sostituiscila con <include file="$(find gazebo_ros)/launch/empty_world_server.launch">
   
