echo heading to home directory
cd
echo redis startup procedure
cd database-server/
echo "starting redis server"
~/database-server/redis-5.0.4/src/redis-server redis.conf & sleep 2
echo "redis server started!"
pkill -9 python
cd ~/TRINA/
echo 'starting server'
cd Motion
python2 motion_server.py & sleep 5
cd ..
echo 'server started'
python3 $PWD/robot_v2/robot.py & sleep 2

python3 UIController_reem.py & 

python2 Robot_visualizer.py &