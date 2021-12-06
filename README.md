## introduction
This is a automous drone demo project. in the demo, the drone will takeoff first, and the fly to scan the code fixed on the wall, if found, the drone will land before the code.
### key word
+ px4
+ rgbd camera
+ obstacle avoidance
+ mapping
+ path planning    
<img src="https://github.com/jinwandou/XJTUCAG_Drone/blob/main/src/support_file/simulation.png" width = "500" height = "500" alt="simulation" align=center /><br/>    
<img src="https://github.com/jinwandou/XJTUCAG_Drone/blob/main/src/support_file/rviz.png" width = "500" height = "500" alt="rviz" align=center /><br/>       
## how to run     
1. set simulation environment first, refer to following website: 
+ [px4 官方user guide](https://docs.px4.io/master/en/simulation/ros_interface.html)   
+ [XTDrone](https://www.yuque.com/xtdrone/manual_cn/basic_config_1.11)    
2. copy src/resource/world/* , src/resource/model/* and src/resource/launch/* to the right location in px4 firmware folder; For example, my px4 firmware folder is ~/PX4_Firmware，corresponding path is as follows：  
```    
automous_drone/src/resource/launch/wooden_mavros_sitl.launch -----------> ~/PX4_Firmware/launch/*    
automous_drone/src/resource/model/* -----------> ~/PX4_Firmware/Tools/sitl_gazebo/models/*    
automous_drone/src/resource/worlds/* -----------> ~/PX4_Firmware/Tools/sitl_gazebo/worlds/*    
```    
3. install dependencies according to README.md in every package;    
4. clone XJTUCAG_Drone to your ros woekspace src folder and use catkin_make to compile all package;    
5. use bash file automous_woodencylinder_sitl.sh to launch all nodes, before use this shell, **edit ros workspace path in the shell to your own path and your ros version**;    
 
