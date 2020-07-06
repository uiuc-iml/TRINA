echo "Killing all python processes"
pkill -9 python
cd ~/TRINA/
echo 'starting server'
cd Motion
python2 motion_server.py & sleep 1
cd ..
echo 'server started'

echo "starting command server"

cd ~/TRINA
python2 command_server.py & sleep 10

echo "command_server_started"

echo "starting GUI"

cd trina_modules/UI

python2 UI_end_1.py