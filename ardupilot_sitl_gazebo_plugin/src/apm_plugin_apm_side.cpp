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
 * Gazebo Plugin:  ardupilot_sitl_gazebo_plugin
 * Description: 
 *   Implements plugin's methods related to the communication with
 *   the SITL interface of Ardupilot.
 *   e.g. covers ports opening, unpacking/packing input/output data.
 */

#include "../include/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin.h"

namespace gazebo
{

//-------------------------------------------------
//  Initialization methods
//-------------------------------------------------

/*
  Initializes variables and ports related to ArduPilot.
  In case of fatal failure, returns 'false'.
 */
bool ArdupilotSitlGazeboPlugin::init_ardupilot_side()
{
    // Setup network infrastructure (opens ports from/to ArduPilot)
    open_control_socket();
    open_fdm_socket();

    return true;
}


//-------------------------------------------------
//  ArduPilot communication - initialization
//-------------------------------------------------

/*
  open control socket from ArduPilot
 */
bool ArdupilotSitlGazeboPlugin::open_control_socket()
{
    if (_is_control_socket_open)
        return true;

    ROS_INFO( PLUGIN_LOG_PREPEND "Binding to listening port from ArduPilot...\n");
    if (!_sock_control_from_ardu->bind("127.0.0.1", PORT_DATA_FROM_ARDUPILOT)) {
        ROS_WARN( PLUGIN_LOG_PREPEND "FAILED to bind to port from ArduPilot\n");
        return false;
    }

    ROS_INFO( PLUGIN_LOG_PREPEND "SUCCESS in binding to port from ArduPilot\n");
    _sock_control_from_ardu->set_blocking(false);
    _sock_control_from_ardu->reuseaddress();
    _is_control_socket_open = true;

    return true;
}

/*
  open fdm socket to ArduPilot
 */
bool ArdupilotSitlGazeboPlugin::open_fdm_socket()
{
    if (_is_fdm_socket_open)
        return true;

    ROS_INFO( PLUGIN_LOG_PREPEND "Connecting send port to ArduPilot...\n");
    if (!_sock_fdm_to_ardu->connect("127.0.0.1", PORT_DATA_TO_ARDUPILOT)) {
        //check_stdout();
        ROS_WARN( PLUGIN_LOG_PREPEND "FAILED to connect to port to ArduPilot\n");
        return false;
    }

    ROS_INFO( PLUGIN_LOG_PREPEND "Opened ArduPilot fdm socket\n");
    _sock_fdm_to_ardu->set_blocking(false);
    _is_fdm_socket_open = true;

    // First message: introduction
    // (may not be received if ArduPilot is not yet running)
    char startup[] = "";
    _sock_fdm_to_ardu->send(startup, strlen(startup));
    return true;
}

    
//-------------------------------------------------
//  ArduPilot communication - running
//-------------------------------------------------

/*
  Receive control inputs from the APM SITL and publishes them in a command/motor_speed topic
 */
bool ArdupilotSitlGazeboPlugin::receive_apm_input()
{
    servo_packet pkt;
    int szRecv;

    if (!_is_control_socket_open) {
        ROS_INFO( PLUGIN_LOG_PREPEND "Cannot receive input from Ardu, for the port is not open !");
        return false;
    }

    szRecv = _sock_control_from_ardu->recv(&pkt, sizeof(pkt), 100);
    // Expects a servo control packet
    if (szRecv != sizeof(servo_packet)) {
        return false;
    }

    // Overwrites the servo control message
    int i;
    bool areAllRotorsOff = true;

    for (i=0; i<_nbMotorSpeed; i++) {
        _cmd_motor_speed[i] = pkt.servos[i] * 1000.0;
        if (areAllRotorsOff && (_cmd_motor_speed[i] > 1))
            areAllRotorsOff = false;
    }

    // When all rotors are off, makes them turn at a very slow pace
    // shows to show that everything works and that the simulation is running
    if (areAllRotorsOff) {
        for (i=0; i<_nbMotorSpeed; i++) {
            // in [rad/s]
            _cmd_motor_speed[i] = 5;     // <=> 0.5 tr/sec
        }
    }

    // Checks if the parachute servo commands a release
    check_parachute_cmd(pkt.servos[SERVO_PARACHUTE]);

    publish_commandMotorSpeed();
    return true;
}


/*
  Packages the fdmState data and sends it to the APM SITL
 */
void ArdupilotSitlGazeboPlugin::send_apm_output()
{
    fdm_packet pkt;

    if (!_is_control_socket_open) {
        ROS_INFO( PLUGIN_LOG_PREPEND "Cannot send output to Ardu, for the port is not open !");
        return;
    }   
    
    // Mutex on '_fdm', for it is concurrently written by ROS callbacks
    _fdm_mutex.lock();
    memcpy(&pkt, &_fdm, sizeof(fdm_packet));
    _fdm_mutex.unlock();
    
    // Makes sure the timestamp is non 0, otherwise Ardupilot can believe it to be an erroneous packet
    if (pkt.timestamp < 1e-6)
        pkt.timestamp = 1e-6;       // 1e-6 [s] = 0.001 [ms]

    ssize_t sent = _sock_fdm_to_ardu->send(&pkt, sizeof(pkt));
}

} // end of "namespace gazebo"
