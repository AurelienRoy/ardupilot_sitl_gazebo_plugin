Recorder Camera Plugin
======================

Purpose
-------
A mobile camera to fix to a robot, or static if added to a .world file,
that continously records a video or frames. It relies on OpenCV for the
file writing. Video files create a .avi output file.
It also emits images in a ROS topic, to check if the scene angle is fine.


Advantages compared to video screen recording
---------------------------------------------
Videos produced with this plugin are synchronised with the simulation time.
So the output video will present the robot running at the same speed as it
would in real life, without slow down/lag issues if the simulation is run on
a slow computer.
Also produced videos can have a higher resolution than the Gazebo window,
or a specific frame rate.


Challenges
----------
The OnNewFrame callback method called by Gazebo is runned in a thread.
So if it takes too long to run, like writing a down the new frame,
the simulation still goes on, and the next callback call may be missed,
thus leading to missed frames.
Therefore the plugin is interfaced with ardupilot_sitl_gazebo_plugin
in order to pause the simulation until the frame writing is done
(use of the laspe-lock).


Issues
------
There seems to be frames lost in the process. Still need to be fixed.


How to use it
-------------
Example for a .sdf file:

      <sensor type="camera" name="static_camera_sensor">
        <camera name="static_cam">
          <horizontal_fov>1.57075</horizontal_fov>
          <image>
            <format>R8G8B8</format>
            <width>1680</width>
            <height>1050</height>
          </image>
          <clip>
            <near>0.01</near>
            <far>200</far>
          </clip>

          <save enabled="false">
            <path>/tmp/gazebo_frames</path>
          </save>
        </camera>

        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>false</visualize>

        <plugin name="static_recorder_camera_plugin" filename="librecorder_camera_plugin.so">
          <saveVideo>true</saveVideo>    <!-- 'true' or 'false' -->
          <videoDir>~/Videos/shots/</videoDir>      <!-- Must already exist -->
          <videoFilename>cam_static</videoFilename>
          <videoFileExt>avi</videoFileExt>    <!-- Only avi is supported by CV2 -->
          <videoCodec>MP4V</videoCodec>   <!-- List on: http://www.fourcc.org/codecs.php DAVC  LMP4   X264-->

          <saveFrames>false</saveFrames>    <!-- 'true' or 'false' -->
          <framesDir>/tmp/gazebo_frames/</framesDir>    <!-- Must already exist -->
          <framesFilename>img_</framesFilename>
          <framesFileExt>png</framesFileExt>

          <imageTopicName>recorder_cam_static</imageTopicName>
        </plugin>
      </sensor>




