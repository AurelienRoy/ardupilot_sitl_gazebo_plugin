
This directory contains all worlds.
Each world is described by a folder containing:
 - one .world file
 - one map image (for MavProxy)


World Folder:
  It must have the exact same name as the .launch file that calls
  this world.

World File:
  Gazebo .world file. It must include the Ardupilot gazebo plugin:
    <plugin name="ardupilot_sitl_gazebo_plugin"
            filename="libardupilot_sitl_gazebo_plugin.so"/>

Map Image:
  MavProxy searches within the folder the first image that matches the
  name pattern "map_<whatever>_w<W>m_h<H>m.<png|jpg|bmp|...>", and loads
  it as an overlay image.
   <W> : width in meters
   <H> : height in meters

