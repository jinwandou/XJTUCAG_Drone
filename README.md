
<img src="https://github.com/jinwandou/XJTUCAG_Drone/blob/main/src/support_file/simulation.png" width = "500" height = "500" alt="simulation" align=center /><br/>    
<img src="https://github.com/jinwandou/XJTUCAG_Drone/blob/main/src/support_file/rviz.png" width = "500" height = "500" alt="rviz" align=center /><br/>       
## user guide     
### set simulation environment    
1. copy src/resource/world/* , src/resource/model/* and src/resource/launch/* to the right location in px4 firmware folder; For example, my px4 firmware folder is ~/PX4_Firmware，corresponding path is as follows：  
```    
automous_drone/src/resource/launch/wooden_mavros_sitl.launch -----------> ~/PX4_Firmware/launch/*    
automous_drone/src/resource/model/* -----------> ~/PX4_Firmware/Tools/sitl_gazebo/models/*    
automous_drone/src/resource/worlds/* -----------> ~/PX4_Firmware/Tools/sitl_gazebo/worlds/*    
```    
2. install dependencies according to README.md in every package;    
3. clone XJTUCAG_Drone to your ros woekspace src folder and use catkin_make to compile all package;    
4. use bash file automous_woodencylinder_sitl.sh to launch all nodes;    
 
