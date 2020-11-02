#! /bin/bash 
##vio自主飞行脚本
#gnome-terminal --window -e 'bash -c "source /opt/ros/kinetic/setup.bash;source ~/project/px4_ws/devel/setup.bash;roscore; exec bash"' \
#--tab -e 'bash -c "sleep 3; cd ~/px4/Firmware;no_sim=1 make posix_sitl_default gazebo; exec bash"' \
#--tab -e 'bash -c "sleep 20; cd ~/px4/Firmware;source ~/project/px4_ws/devel/setup.bash;source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default;export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd);export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo;roslaunch px4 posix_sitl.launch; exec bash"' \

gnome-terminal --window -e 'bash -c "source /opt/ros/kinetic/setup.bash;source ~/project/px4_ws/devel/setup.bash;roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; cd ~/PX4_Firmware;roslaunch px4 posix_sitl.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; source ~/project/px4_ws/devel/setup.bash;roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" gcs_url:=udp-b://@; exec bash"' \
--tab -e 'bash -c "sleep 8; source ~/project/px4_ws/devel/setup.bash;roslaunch px4_autonomy landpad_det_gazebo.launch; exec bash"' \
--tab -e 'bash -c "sleep 9; source ~/project/px4_ws/devel/setup.bash;roslaunch px4_autonomy offboard_control.launch; exec bash"' \

#--tab -e 'bash -c "sleep 2; source ~/project/px4_ws/devel/setup.bash;roslaunch realsense2_camera rs_t265.launch; exec bash"' \
#--tab -e 'bash -c "sleep 2; source ~/project/px4_ws/devel/setup.bash;roslaunch vision_to_mavros t265_tf_to_mavros.launch; exec bash"' \
