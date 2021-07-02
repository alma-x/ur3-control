#!/bin/bash
cd
echo PULLING ALL
cd ../catkin_ws/src/ur3-control
git pull
cd ../simulation-cv
git pull
cd ../almasim_ur3
git pull
cd ../ERC_2021_simulator
git pull
cd ../..
catkin_make
echo FINISH TO PULL
