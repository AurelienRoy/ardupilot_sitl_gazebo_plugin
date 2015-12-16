cmake_minimum_required(VERSION 2.8.3)
project(recorder_camera_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
   roscpp
   gazebo_ros
   cv_bridge
   image_transport
   sensor_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
   SHARED
   src/recorder_camera_plugin.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
    DEPENDS
      roscpp
      gazebo_ros
      cv_bridge
      image_transport
      sensor_msgs
)

#    CATKIN_DEPENDS nodelet image_transport dynamic_reconfigure driver_base gazebo_plugins
#    INCLUDE_DIRS include
#    LIBRARIES


