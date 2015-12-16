cmake_minimum_required(VERSION 2.8.3)
project(ardupilot_sitl_gazebo_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros
  rotors_gazebo_plugins
  glog_catkin
  geometry_msgs
  mav_msgs
  sensor_msgs
  std_msgs
  message_generation
)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

# Depend on system install of Gazebo
#find_package(gazebo REQUIRED)

#link_directories(${GAZEBO_LIBRARY_DIRS})
#include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
    ReleaseApmLapseLock.srv
    TakeApmLapseLock.srv
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    rotors_gazebo_plugins
    glog_catkin
    geometry_msgs
    mav_msgs
    sensor_msgs
	aircraft_plugin
    message_runtime
  INCLUDE_DIRS
    include
)


###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/ardupilot_sitl_gazebo_plugin.cpp
  src/apm_plugin_apm_side.cpp
  src/apm_plugin_gazebo_side.cpp
  src/apm_plugin_ros_side.cpp
  src/apm_plugin_parachute.cpp
  src/SocketAPM.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)


