Ardupilot SITL Gazebo Plugin
============================

This plugin is an interface between Gazebo and Ardupilot.


BUILD
-----
Move to the right folder with in a terminal:
  roscd
  cd ..

Then type in:
  catkin_make


RUNNING
-------
In a terminal, from anywhere, type:
  roslaunch ardupilot_sitl_gazebo_plugin <launch_file>

  with <launch_file> referencing a .launch file within the 'launch' folder,
  for example 'empty_world.launch'.

It is advised to run it using the script sim_vehicle.sh of Ardupilot.
(check you have the right version that includes ROS related modifications!)


ACKNOWLEDGEMENTS
----------------

Some material within this plugin come from the plugin rotors_simulator
from ETHZ. They have been copied here for they needed to be modified.
There are still a some links to the rotors_simulator files, like
'component_snippets.xacro'.
Modified urdf files have been included in the project, to avoid additional
forks of the rotors_simulator project.

The root of the plugin was a Python ROS plugin, written by Alexander BUYVAL.
Thanks to him for sharing his setup configuration and Python interface.


AUTHORS
-------
Aurélien ROY
Maxime LAFLEUR


ARDUPILOT
---------
The plugin currently supports:
   ArduCopter
   ArduPlane


ISSUES
------
The world 'outdoor_village' makes use of a heightmap for the mountains.
However there is a Gazebo bug when using ray sensors (= range finders) on
heightmaps, and running performance drastically drop. The bug depends on both
the size of the heightmap image and its scalling.
A solution is to convert the terrain heightmap into a triangle mesh.


PARACHUTE
---------
To make the parachute work, define your autopilot parameters as :
  CHUTE_ALT_MIN    0         removes the altitude constraint in the simulation
  CHUTE_ENABLED    1
  CHUTE_SERVO_OFF  1000
  CHUTE_SERVO_ON   2000      the Gazebo plugin triggers the parachute if the servo position is > 1500
  CHUTE_TYPE       10        to release with a servo
  RC10_FUNCTION    27        for the output channel n°9 on which the Gazebo plugin expects the trigger

And yes the RC10 for output channel n°9 is a bit strange.

Arm the motors, takeoff, and in mid-air, type the command "parachute release" on
the MavProxy terminal.
(The command appeared recently, so if it is unknwon, you may have to update your MavProxy version)


DFFICULTIES ENCOUTERED
----------------------

 * on a slow computer, ArduCopter was running either too slowly, or too fast
   compared to the simulation.
   When it was too slowly, the simulation returned a time step dt of 0, and
   it was throwing a FPE erorr (Float Point Error for 0 division) from somewhere
   in Arducopter's code.
   When Arducopter was running too fast, it was missing some simulation steps.
   All this was resolved using the Step(1) Gazebo function. But it required to
   transform the ROS plugin into a Gazebo plugin.

 * Frequent disconnections of MavProxy and Mavros (no link error)
   Cause:
    1 During the gyro bias calibration, ardupilot does not send mavlink heartbeat
      messages.
    2 And ardupilot was stuck on the gyro calib because it was seing large
      accelerometer variances, and thinks the UAV was moved.
    3 Accelerometer measures are computed in the rotors_gazebo_plugins, 
      inside "gazebo_imu_plugin.cpp". The calculation is a simple Euler
      differentiation scheme, which is not very robust to noise on the input
      velocity.
    4 The velocity was very noisy because the UAV was bouncing off the ground
      plane by +/- 0.001 mm on each step, because of ODE's interpenetration.
      Even if the movement was very small (not eye visible on Gazebo view),
      differentiated by a 2.5 ms, it resulted in +/- 10 Gs on each step.
    5 ODE's interpenetration was due a poorly chosen relaxation factor (SOR),
      in Gazebo .world file, replacing <sor>1.3</sor> by <sor>1.0</sor>
      fixed the problem.
      
    I tried solving the problem at step 4, by replacing the simple Euler
    differentiation with one of Pavel Holoborodko's smooth noise-robust
    one-sided differentiators. It helped, but was not enough to conteract
    the full problem.
    (this chain reaction of problems, starting from ODE's SOR parameter
     to end up in ground station link loss, is quite a thing ^^)


