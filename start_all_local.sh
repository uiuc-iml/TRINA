echo "Killing all python processes"
pkill -9 python
cd ~/TRINA/
echo 'starting server'
cd Motion
python2 motion_server.py & sleep 1
cd ..
echo 'server started'

echo "starting command server"

python3 $PWD/robot_v2/robot.py & sleep 2

python3 UIController_reem.py & 

python3 Robot_visualizer.py &
