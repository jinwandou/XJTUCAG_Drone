#! /bin/bash 
##vio自主飞行脚本
gnome-terminal --window -e 'bash -c "source /opt/ros/kinetic/setup.bash;source ~/project/px4_ws/devel/setup.bash;roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; source ~/project/px4_ws/devel/setup.bash;roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 gcs_url:=udp-b://@; exec bash"' \
--tab -e 'bash -c "sleep 2; source ~/project/px4_ws/devel/setup.bash;roslaunch realsense2_camera rs_t265.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; source ~/project/px4_ws/devel/setup.bash;roslaunch vision_to_mavros t265_tf_to_mavros.launch; exec bash"' \

#--tab -e 'bash -c "sleep 2; source ~/project/px4_ws/devel/setup.bash;roslaunch px4_command px4_pos_estimator.launch; exec bash"' \
#--tab -e 'bash -c "sleep 2; source ~/project/px4_ws/devel/setup.bash;roslaunch px4_command px4_pos_controller.launch; exec bash"' \
#--tab -e 'bash -c "sleep 2; source ~/project/px4_ws/devel/setup.bash;rosrun px4_command move; exec bash"' \
