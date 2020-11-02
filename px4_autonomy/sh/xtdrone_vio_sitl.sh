#! /bin/bash 
##vio xtdrone sitl
gnome-terminal --window -e 'bash -c "source /opt/ros/kinetic/setup.bash;source ~/project/workspace/devel/setup.bash;roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; cd ~/PX4_Firmware;roslaunch px4 indoor1.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/project/workspace/devel/setup.bash;bash ~/project/workspace/scripts/xtdrone_vins_run.sh;exec bash"' \
--tab -e 'bash -c "sleep 5; cd ~/project/XTDrone/sensing/slam/vio;python vins_transfer.py iris; exec bash"' \
--tab -e 'bash -c "sleep 5; cd ~/project/XTDrone/communication;python multirotor_communication.py iris 0; exec bash"' \
--tab -e 'bash -c "sleep 5; cd ~/project/XTDrone/control;python multirotor_keyboard_control.py iris 1 vel; exec bash"'
