#!/bin/bash
cd
cd scripts/git_scripts
echo PUSHING ALL
./git_pushing_ERC_simulator.sh
./git_pushing_simulation_cv.sh
./git_pushing_ur3.sh
git config --global credential.helper 'cache --timeout=36000000'
echo FINISH TO PUSH
