#!/bin/bash
cd ~/TRINA/
cd Motion
### Comment this out if you do not want to kill all other python processes running on your computer
pkill -9 python
echo "starting the motion server"
python2 motion_server.py & sleep 4
echo "starting control GUI"
python2 controlGUI.py