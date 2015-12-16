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



#include "../include/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin.h"
#include <cstdlib>

// TODO: find a cleaner way to ge the robot namespace
#define ROBOT_NAMESPACE     "iris"
//#define ROBOT_NAMESPACE     "cessna"


namespace gazebo
{
    
//-------------------------------------------------
//  Initialization methods
//-------------------------------------------------

/*
  Initializes variables and declares subscribers/publisher related to ROS.
  In case of fatal failure, returns 'false'.
 */
bool ArdupilotSitlGazeboPlugin::init_ros_side()
{
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized()) {
        ROS_FATAL_STREAM( PLUGIN_LOG_PREPEND "A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return false;
    }
    
    // Setup ROS node infrastructure
    _rosnode = new ros::NodeHandle(ROS_NAMESPACE);
    
    // Defines topics callback methods
    std::string topicNameBuf;

    // IMU topic (noise free)
    topicNameBuf = std::string("/") + _modelName + "/ground_truth/imu";
    _imu_subscriber = _rosnode->subscribe(topicNameBuf.c_str(), 1, &ArdupilotSitlGazeboPlugin::imu_callback, this);

    // GPS topic
    topicNameBuf = std::string("/") + _modelName + "/fix";
    _gps_subscriber = _rosnode->subscribe(topicNameBuf.c_str(), 1, &ArdupilotSitlGazeboPlugin::gps_callback, this);

    // GPS velocity topic
    topicNameBuf = std::string("/") + _modelName + "/fix_velocity";
    _gps_velocity_subscriber = _rosnode->subscribe(topicNameBuf.c_str(), 1, &ArdupilotSitlGazeboPlugin::gps_velocity_callback, this);

    _sonar_down_subscriber   = _rosnode->subscribe("/sonar_down", 1, &ArdupilotSitlGazeboPlugin::sonar_down_callback,   this);
#if SONAR_FRONT == ENABLED
    _sonar_front_subscriber  = _rosnode->subscribe("/sonar_front", 1, &ArdupilotSitlGazeboPlugin::sonar_front_callback,  this);
#endif
    
    // Notes:
    //  - Uses the truth IMU, with a 1-size queue. All noises will be added by Ardupilot
    //  - For the noisy imu, use the topic "/iris/imu".
    //  - No need to subscribe to ROS's clock topic, "/clock", for we use Gazebo's clock
    
    // Buffer size of 10 messages before old ones are removed
    topicNameBuf = std::string("/") + _modelName + "/command/motor_speed";
    _motorSpd_publisher = _rosnode->advertise<mav_msgs::CommandMotorSpeed>(topicNameBuf.c_str(), 10);
    
    // Services
    _service_take_lapseLock    = _rosnode->advertiseService("take_apm_lapseLock",    &ArdupilotSitlGazeboPlugin::service_take_lapseLock,    this);
    _service_release_lapseLock = _rosnode->advertiseService("release_apm_lapseLock", &ArdupilotSitlGazeboPlugin::service_release_lapseLock, this);
    ROS_INFO( PLUGIN_LOG_PREPEND "Services declared !");
      
    return true;
}


//-------------------------------------------------
//  ROS Services
//-------------------------------------------------

bool ArdupilotSitlGazeboPlugin::service_take_lapseLock(ardupilot_sitl_gazebo_plugin::TakeApmLapseLock::Request  &req,
                                                       ardupilot_sitl_gazebo_plugin::TakeApmLapseLock::Response &res)
{
    ROS_DEBUG( PLUGIN_LOG_PREPEND "service_take_lapseLock: called by process '%s'", req.process_id.c_str());
    take_lapseLock(req.max_duration);
    res.nb_holders = _nbHolders_lapseLock;
    return true;
}

  
bool ArdupilotSitlGazeboPlugin::service_release_lapseLock(ardupilot_sitl_gazebo_plugin::ReleaseApmLapseLock::Request  &req,
                                                          ardupilot_sitl_gazebo_plugin::ReleaseApmLapseLock::Response &res)
{
    ROS_DEBUG( PLUGIN_LOG_PREPEND "service_release_lapseLock: called by process '%s'", req.process_id.c_str());
    release_lapseLock();
    res.nb_holders = _nbHolders_lapseLock;
    return true;
}

//-------------------------------------------------
//  ROS Topics Listeners
//-------------------------------------------------

/*
  Callback method for ROS messages ".../imu", coming from an IMU sensor
 */
void ArdupilotSitlGazeboPlugin::imu_callback(const sensor_msgs::Imu &imu_msg)
{
    // This method is executed independently from the main loop thread.
    // Beware of access to shared variables memory. Use mutexes if required.
    
    // Frame conversion:
    //   IMU messages arrive expressed in Gazebo frame (Y toward West, Z toward UP).
    //   So values are converted to NED frame before sending them to Ardupilot.
    
    // Mutex on '_fdm', for it is concurrently read by 'send_apm_output()'
    _fdm_mutex.lock();
    
    // Attitude (quaternion)
    _fdm.imu_orientation_quat[0] =  imu_msg.orientation.w;
    _fdm.imu_orientation_quat[1] =  imu_msg.orientation.x;
    _fdm.imu_orientation_quat[2] = -imu_msg.orientation.y;
    _fdm.imu_orientation_quat[3] = -imu_msg.orientation.z;
    
    // Angular velocity
    _fdm.imu_angular_velocity_rpy[0] =  imu_msg.angular_velocity.x;    // [rad/s]
    _fdm.imu_angular_velocity_rpy[1] = -imu_msg.angular_velocity.y;    // [rad/s]
    _fdm.imu_angular_velocity_rpy[2] = -imu_msg.angular_velocity.z;    // [rad/s]

    // Acceleration
    _fdm.imu_linear_acceleration_xyz[0] =  imu_msg.linear_acceleration.x;    // [m/s/s]
    _fdm.imu_linear_acceleration_xyz[1] = -imu_msg.linear_acceleration.y;    // [m/s/s]
    _fdm.imu_linear_acceleration_xyz[2] = -imu_msg.linear_acceleration.z;    // [m/s/s]
    
    // Position in the Gazebo world (NOT HANDLED YET)
    _fdm.position_xyz[0] = 0;    // [m]
    _fdm.position_xyz[1] = 0;    // [m]
    _fdm.position_xyz[2] = 0;    // [m]
    
    _fdm_mutex.unlock();
}

/*
  Callback method for ROS messages ".../fix", coming from a GPS sensor
 */
void ArdupilotSitlGazeboPlugin::gps_callback(const sensor_msgs::NavSatFix &gps_fix_msg)
{
    // This method is executed independently from the main loop thread.
    // Beware of access to shared variables memory. Use mutexes if required.
    
    // Mutex on '_fdm', for it is concurrently read by 'send_apm_output()'
    _fdm_mutex.lock();
    
    _fdm.position_latlonalt[0] = gps_fix_msg.latitude;     // in [degrees]
    _fdm.position_latlonalt[1] = gps_fix_msg.longitude;    // in [degrees]
    _fdm.position_latlonalt[2] = gps_fix_msg.altitude;     // in [m], toward UP
    
    _fdm_mutex.unlock();
    
    // Available display for GPS debug
    #ifdef DEBUG_DISP_GPS_POSITION
        static double home_lat = 0, home_lon = 0;       // a home for debug
        double north_m, east_m, r = 6371000;
        
        // The relative position in meters is calculated from the first non-0
        // GPS position. (So it will be equivalent to the starting point of the UAV)
        if ((abs(home_lat) < 1e-5) && (abs(home_lon) < 1e-5) &&
        	((abs(gps_fix_msg.latitude) > 1e-4) || (abs(gps_fix_msg.longitude) > 1e-4))) {
        	home_lat = gps_fix_msg.latitude;
        	home_lon = gps_fix_msg.longitude;
        }

        north_m = r * sin(gps_fix_msg.latitude - home_lat);
        east_m = r * cos(gps_fix_msg.latitude) * sin(gps_fix_msg.longitude - home_lon);
        
        // Latitude & longitude can be tricky to interpret in a debug log.
        // That is why the home debug was introduced, to display results in a more readable coordinate system.
        ROS_INFO( PLUGIN_LOG_PREPEND "GPS latitude = %f [d] - %f [m]   longi = %f [d] - %f [m]   alt = %f [m]", gps_fix_msg.latitude, north_m, gps_fix_msg.longitude, east_m, gps_fix_msg.altitude);
    #endif // DEBUG_DISP_GPS_POSITION
}

/*
  Callback method for ROS messages ".../fix_velocity", coming from a GPS sensor
 */
void ArdupilotSitlGazeboPlugin::gps_velocity_callback(const geometry_msgs::Vector3Stamped &gps_velocity_fix_msg)
{
    // This method is executed independently from the main loop thread.
    // Beware of access to shared variables memory. Use mutexes if required.
    
    // Mutex on '_fdm', for it is concurrently read by 'send_apm_output()'
    _fdm_mutex.lock();
    
    // in Hector's plugin, the GPS velocity Y is toward West, not East. And velocity Z is toward UP
    _fdm.velocity_xyz[0] =  gps_velocity_fix_msg.vector.x;    // in [m/s]
    _fdm.velocity_xyz[1] = -gps_velocity_fix_msg.vector.y;    // in [m/s]
    _fdm.velocity_xyz[2] = -gps_velocity_fix_msg.vector.z;    // in [m/s]
    
    // Available display for GPS speed debug
    #ifdef DEBUG_DISP_GPS_POSITION
        ROS_INFO( PLUGIN_LOG_PREPEND "GPS speed N = %f   E = %f   D = %f", _fdm.velocity_xyz[0], _fdm.velocity_xyz[1], _fdm.velocity_xyz[2]);
    #endif // DEBUG_DISP_GPS_POSITION
    
    _fdm_mutex.unlock();
}

/*
  Callback method for ROS messages ".../sonar_down", coming from a range finder sensor
 */
void ArdupilotSitlGazeboPlugin::sonar_down_callback(const sensor_msgs::Range &sonar_range_msg) 
{
    // This method is executed independently from the main loop thread.
    // Beware of access to shared variables memory. Use mutexes if required.
    
    // Mutex on '_fdm', for it is concurrently read by 'send_apm_output()'
    _fdm_mutex.lock();
    
    _fdm.sonar_down = sonar_range_msg.range;    // in [m]
    
    _fdm_mutex.unlock();
}

/*
  Callback method for ROS messages ".../sonar_front", coming from a range finder sensor
 */
#if SONAR_FRONT == ENABLED
void ArdupilotSitlGazeboPlugin::sonar_front_callback(const sensor_msgs::Range &sonar_range_msg)
{
    // This method is executed independently from the main loop thread.
    // Beware of access to shared variables memory. Use mutexes if required.
    
    // Mutex on '_fdm', for it is concurrently read by 'send_apm_output()'
    _fdm_mutex.lock();
    
    _fdm.sonar_front = sonar_range_msg.range;    // in [m]
    
    _fdm_mutex.unlock();
}
#endif


//-------------------------------------------------
//  ROS Topics Publishers
//-------------------------------------------------

/*
  Fill and publish motor speed command data message
 */
void ArdupilotSitlGazeboPlugin::publish_commandMotorSpeed()
{
    boost::shared_ptr<mav_msgs::CommandMotorSpeed> cmdMotSpd_msg = boost::make_shared<mav_msgs::CommandMotorSpeed>();
    //auto cmdMotSpd_msg = boost::make_shared<mav_msgs::CommandMotorSpeed>();
    
    // The 'auto' keyword is used in some plugins with the 'boost::make_shared'.
    // It is a recent (new) feature since 2014 in g++ compilers.
    // It did not work with my version, so I preferred to use the old
    // way with an explicit type declaration.
    
    int i;
    std::string frame_id = "quad";
    
    cmdMotSpd_msg->header.frame_id = frame_id;
    cmdMotSpd_msg->header.stamp = ros::Time::now();
    cmdMotSpd_msg->motor_speed.clear();
    for (i=0; i<_nbMotorSpeed; i++)
       cmdMotSpd_msg->motor_speed.push_back(_cmd_motor_speed[i]);
    
    _motorSpd_publisher.publish(cmdMotSpd_msg);
    
    // Available display for motors commands debug
    #ifdef DEBUG_DISP_MOTORS_COMMANDS
        static float s_debug_cmd_motor_speed_prev[_nbMotorSpeed];
        bool isDiffFromPrev = false;
        
        for (i=0; i<_nbMotorSpeed; i++) {
            if (abs(s_debug_cmd_motor_speed_prev[i] - _cmd_motor_speed[i]) > 0.001) {
                isDiffFromPrev = true;
                s_debug_cmd_motor_speed_prev[i] = _cmd_motor_speed[i];
                // Do not break, to continue updating every field
            }
        }
        
        if (isDiffFromPrev) {
            ROS_INFO( PLUGIN_LOG_PREPEND "published motor speed: %f, %f, %f, %f", _cmd_motor_speed[0],
                  _cmd_motor_speed[1], _cmd_motor_speed[2], _cmd_motor_speed[3]);
        }
    #endif // DEBUG_DISP_MOTORS_COMMANDS
}

} // end of "namespace gazebo"
