#!/bin/bash

cd workspace/src/aichallenge_submit/local_planner_custom/local_planner_custom/GraphBasedLocalTrajectoryPlanner/
pip3 install -r requirements.txt 
pip3 uninstall quadprog -y
pip3 install quadprog
pip3 install -U .