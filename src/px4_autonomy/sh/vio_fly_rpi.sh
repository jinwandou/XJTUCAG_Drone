#! /bin/bash 
## automous fly with vio running on rpi 3b+

mate-terminal --window -e 'bash -c "source /opt/ros/kinetic/setup.bash;source ~/catkin_ws/devel/setup.bash;roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/catkin_ws/devel/setup.bash;roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 gcs_url:=udp-b://@; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/catkin_ws/devel/setup.bash;roslaunch realsense2_camera rs_t265.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/catkin_ws/devel/setup.bash;roslaunch vision_to_mavros t265_tf_to_mavros.launch; exec bash"' \

