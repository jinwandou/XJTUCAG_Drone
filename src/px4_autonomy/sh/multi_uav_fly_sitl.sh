#! /bin/bash 
## multi uav自主飞行脚本
gnome-terminal --window -e 'bash -c "source /opt/ros/kinetic/setup.bash;source ~/project/px4_ws/devel/setup.bash;roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; cd ~/px4/Firmware;source ~/project/px4_ws/devel/setup.bash;source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd);export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo;roslaunch px4_command three_uav_mavros_sitl.launch; exec bash"' \
#--tab -e 'bash -c "sleep 2; source ~/project/px4_ws/devel/setup.bash;roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
#--tab -e 'bash -c "sleep 2; source ~/project/px4_ws/devel/setup.bash;roslaunch px4_command px4_pos_controller.launch; exec bash"' \
#--tab -e 'bash -c "sleep 2; source ~/project/px4_ws/devel/setup.bash;rosrun px4_command move; exec bash"' \

