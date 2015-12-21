/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Authors:
 *   Aurelien ROY
 *   Maxime LAFLEUR
 *
 * Acknowledgments:
 *   to Alexander BUYVAL, for sharing his Gazebo setup and Python interface which
 *   was the root of this project
 */

// Future improvements
// this->physicsEngine->SetSeed(_msg->seed());

#ifndef ARDUPILOT_SITL_GAZEBO_PLUGIN_H
#define ARDUPILOT_SITL_GAZEBO_PLUGIN_H


// Gazebo includes
#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/util/system.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"

// ROS includes
#include <ros/ros.h>
#include <ros/console.h>

// Network
#include <sys/socket.h>
#include <netinet/in.h>

// ROS messages
#include <mav_msgs/CommandMotorSpeed.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>              // for the sonar
#include <sensor_msgs/NavSatFix.h>          // for the GPS fix
#include <geometry_msgs/Vector3Stamped.h>   // for the GPS velocity fix
#include <std_msgs/Empty.h>

// This plugin implements a thread, based on boost, for the communication with Ardupilot
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// Standard includes
#include <string>
#include <stdio.h>

// Plugin's inner headers
#include "SocketAPM.h"

// Plugin's services
#include "ardupilot_sitl_gazebo_plugin/TakeApmLapseLock.h"
#include "ardupilot_sitl_gazebo_plugin/ReleaseApmLapseLock.h"


//--------------------------------------------
// Sensors configuration
#define ENABLED      1
#define DISABLED     0

#define SONAR_FRONT    ENABLED

//--------------------------------------------
// URDF/XACRO models descriptions names
#define UAV_MODEL_NAME                 "iris"
#define UAV_MODEL_CG_LINK              "base_link"

#define PARACHUTE_MODEL_NAME           "parachute_small"
#define PARACHUTE_MODEL_ATTACH_LINK    "chute"

// All ROS topics emitted by this plugin with have the
// namespace "/fdmUDP" prepended.
#define ROS_NAMESPACE     "/fdmUDP"


//--------------------------------------------
// Communication with Ardupilot

#define PORT_DATA_FROM_ARDUPILOT       9002
#define PORT_DATA_TO_ARDUPILOT         9003

// Messages passed
#define NB_SERVOS                 16
#define NB_SERVOS_MOTOR_SPEED     4            // 4(iris) or 5(cessna)

#define SERVO_PARACHUTE           9

#define STEP_SIZE_FOR_ARDUPILOT    0.0025      // in [s], = 400 Hz



#define MAX_LAPSE_LOCK_ON_MODEL_INSERT   5     // [s]
#define MAX_LAPSE_LOCK_DEFAULT           1     // [s]


//--------------------------------------------
// Math
#define PI      3.1415926536
#define PI_2    1.5707963268

//--------------------------------------------
// Log/Debug
#define PLUGIN_LOG_PREPEND       "ARI: "

// Uncomment the following lines to activate some debug
//#define DEBUG_DISP_GPS_POSITION
//#define DEBUG_DISP_MOTORS_COMMANDS



namespace gazebo
{

class ArdupilotSitlGazeboPlugin : public WorldPlugin
{
  public:
        
    // Constructor
    ArdupilotSitlGazeboPlugin();
    ~ArdupilotSitlGazeboPlugin();
    
    // Public function members
    void Load(physics::WorldPtr world, sdf::ElementPtr sdf);
          
    // MAIN LOOP related methods ---------------
    void take_lapseLock(float max_holder_lock_duration = MAX_LAPSE_LOCK_DEFAULT);
    bool release_lapseLock();
    
    // GAZEBO related methods ------------------
    void on_gazebo_update();
    void on_gazebo_control(ConstWorldControlPtr &_msg);
    void on_gazebo_modelInfo(ConstModelPtr &_msg);
  
    // ROS related methods ---------------------
    void imu_callback(const sensor_msgs::Imu &imu_msg);
    void gps_callback(const sensor_msgs::NavSatFix &gps_fix_msg);
    void gps_velocity_callback(const geometry_msgs::Vector3Stamped &gps_velocity_fix_msg);
      void sonar_down_callback(const sensor_msgs::Range &sonar_range_msg);
  #if SONAR_FRONT == ENABLED
    void sonar_front_callback(const sensor_msgs::Range &sonar_range_msg);
  #endif
    
    // Services:
    bool service_take_lapseLock(ardupilot_sitl_gazebo_plugin::TakeApmLapseLock::Request  &req,
                                ardupilot_sitl_gazebo_plugin::TakeApmLapseLock::Response &res);
    bool service_release_lapseLock(ardupilot_sitl_gazebo_plugin::ReleaseApmLapseLock::Request  &req,
                                   ardupilot_sitl_gazebo_plugin::ReleaseApmLapseLock::Response &res);

    
    
  protected:
    /*
      packet sent to ardupilot_sitl_gazebo
     */
    struct servo_packet {
        float servos[NB_SERVOS];    // ranges from 0 (no rotation) to 1 (full throttle)
    };

    /*
      reply packet sent from Gazebo to ArduPilot
     */
    struct fdm_packet {
      double timestamp;                             // [seconds] simulation time
      double imu_angular_velocity_rpy[3];           // [rad/s]
      double imu_linear_acceleration_xyz[3];        // [m/s/s] in NED, body frame
      double imu_orientation_quat[4];               // rotation quaternion, APM conventions, from body to earth
      double velocity_xyz[3];                       // [m/s] in NED
      double position_xyz[3];                       // [m] in NED, from Gazebo's map origin (0,0,0)
      double position_latlonalt[3];                 // [degrees], altitude is Up

      // You can add here extra sensors to pass along
      double sonar_down;                            // [m] downward facing range finder
      
    #if SONAR_FRONT == ENABLED
      double sonar_front;                           // [m] forward facing range finder
    #endif
      
    };
    
    // Initialization methods ------------------
    bool init_ros_side();
    bool init_gazebo_side(physics::WorldPtr world, sdf::ElementPtr sdf);
    bool init_ardupilot_side();
      
    // MAIN LOOP related methods ---------------
    void loop_thread();
    bool check_lapseLock(float loop_elapsed_dt);
    void clear_lapseLock();

    // ARDUPILOT related methods --------------
    bool open_control_socket();
    bool open_fdm_socket();
    bool receive_apm_input();
    void send_apm_output();

    
    // GAZEBO related methods ------------------
    void step_gazebo_sim();
    void check_parachute_cmd(float servo_parachute);
    void load_parachute_model();
    void on_parachute_model_loaded();
  
    
    
    // ROS related methods ---------------------
    void quat_to_euler(float q1, float q2, float q3, float q4,
                       float &roll, float &pitch, float &yaw);
    void publish_commandMotorSpeed();
    
    
    // Node Handles
    
    // Gazebo messages / data
    gazebo::physics::WorldPtr   _parent_world;
    gazebo::physics::ModelPtr   _uav_model;
    gazebo::physics::ModelPtr   _parachute_model;
    gazebo::physics::LinkPtr    _chute_link_on_uav;
    gazebo::physics::JointPtr   _uav_chute_joint;
    sdf::ElementPtr             _sdf;
    transport::SubscriberPtr    _controlSub;      // Subscriber used to receive updates on world_control topic.
    transport::SubscriberPtr    _modelInfoSub;
  
    bool                        _isSimPaused;     // Flag to hold the simulation (pause)
    bool                        _timeMsgAlreadyDisplayed;  // for debug of the time step
    bool                        _is_parachute_available;
    
    // ROS messages
    ros::NodeHandle*            _rosnode;
            
    ros::ServiceServer          _service_take_lapseLock;
    ros::ServiceServer          _service_release_lapseLock;
    
    ros::Subscriber             _imu_subscriber;
    ros::Subscriber             _sonar_down_subscriber;
    ros::Subscriber             _sonar_front_subscriber;
    ros::Subscriber             _gps_subscriber;
    ros::Subscriber             _gps_velocity_subscriber;
    
    ros::Publisher              _motorSpd_publisher;
        
    float                       _cmd_motor_speed[NB_SERVOS_MOTOR_SPEED];    // Local copy of the motor speed command, in [rad/s]
    
    // Vehicle parameters
    std::string                 _modelName;
    int 			            _nbMotorSpeed;
    
    // Timing
    ros::Duration               _control_period;
    ros::Time                   _last_update_sim_time_ros;
    ros::Time                   _last_write_sim_time_ros;
    
    event::ConnectionPtr        _updateConnection;
  
    SocketAPM *_sock_fdm_to_ardu;
    SocketAPM *_sock_control_from_ardu;

    bool _is_control_socket_open;
    bool _is_fdm_socket_open;

    fdm_packet                  _fdm;
    boost::mutex                _fdm_mutex;
    
    boost::thread               _callback_loop_thread;
    
    // LapseLock:
    //  A calling process can block the main loop from running, for a specified maximum time (wall-time, not sim time).
    //  The main loop is resumed if the calling process releases the lock, of if the time has elapsed.
    //  It is used for:
    //    - Dynamic model insertion: Model loading in Gazebo is performed in a parallel thread.
    //      Pauses the simulation until the model is loaded.
    //    - Camera recorder: ROS/Gazebo calls plugins' callback methods in threads, to ensure a fast
    //      running loop. The message queue system is supposed to handle the few lingerings that might
    //      happen. However if the plugin is known to not be able to run at real-time, then it should
    //      make use of the lapse-lock to slow down the simulation. For example, the camera recorder plugin.
    
    // Identified limit of LapseLock:
    //  Currently their is no memory/identification of processes holding the lapse-lock, only a counter.
    //  With this system issues might arise when the following situation occurs:
    //
    //      process A takes lapse-lock for 5 seconds, meanwhile it makes a long computation (e.g. 8 seconds long)
    //      simulation is paused (lapse-lock > 0)
    //      ... 5 seconds pass on wall clock ...
    //      simulation is resumed (it resets nbHolders to 0)
    //      process B takes lapse-lock for 5 seconds, meanwhile it makes a long computation -> nbHolders = 1
    //      simulation is paused (lapse-lock > 0)
    //      process A finishes its computation, and calls the release method -> nbHolders = 0 -> _loop_lapseLock = 0
    //      simulation is resumed, however process B is not finished !
    
    float                       _loop_lapseLock;            // [s] blocks the main loop until release (-> 0), or if the timer is elapsed
    int                         _nbHolders_lapseLock;       // [1] number of calling processes to the lapselock (case of multiple lockers)
    boost::mutex                _lapseLock_mutex;           // to protect the lapse-lock from simulatenous modifications
    
};

}   // end of namespace gazebo


#endif // ARDUPILOT_SITL_GAZEBO_PLUGIN_H

