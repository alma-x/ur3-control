# ur3-control

https://github.com/PUT-UGV-Team/ur3_erc_docker


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
