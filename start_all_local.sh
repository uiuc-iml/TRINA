echo heading to home directory
source ~/trina_env/bin/activate
cd ~/
echo redis startup procedure
cd database-server/
echo "starting redis server"
~/database-server/redis-5.0.4/src/redis-server redis.conf & sleep 2
echo "redis server started!"
pkill -9 python
cd ~/TRINA/
echo 'starting server'
cd Motion
python2 motion_server.py & sleep 1
cd ..
echo 'server started'
python $PWD/robot_v2/robot.py & sleep 2

python UIController_reem.py & 

python2.7 Robot_visualizer.py &
