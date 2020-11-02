cmake_minimum_required(VERSION 2.8.3)
project(px4_autonomy)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 message_generation
 roscpp
 geometry_msgs
 sensor_msgs
 mavros
 nav_msgs
 std_msgs
 std_srvs
 tf2_ros
 tf2_eigen
 mavros_msgs
 cv_bridge
 image_transport
 pcl_ros
 pcl_conversions
 prometheus_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

find_package( PCL REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )

generate_messages(
  DEPENDENCIES
  geometry_msgs 
  sensor_msgs
  std_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
catkin_package(
  CATKIN_DEPENDS  message_runtime
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  src/common/include
)

###########
## Build ##
###########
#add_executable(offboard_control src/control/offboard_control.cpp)
#add_dependencies(offboard_control px4_autonomy_gencpp)
#target_link_libraries(offboard_control ${catkin_LIBRARIES})

add_executable(landpad_det src/perception/object_detection/cpp_nodes/landpad_det.cpp)
add_dependencies(landpad_det px4_autonomy_gencpp)
target_link_libraries(landpad_det ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

#add_executable(autonomous_landing src/control/autonomous_landing/autonomous_landing.cpp)
#add_dependencies(autonomous_landing px4_autonomy_gencpp)
#target_link_libraries(autonomous_landing ${catkin_LIBRARIES})
#############
## Install ##
#############
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_px4_autonomy.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
