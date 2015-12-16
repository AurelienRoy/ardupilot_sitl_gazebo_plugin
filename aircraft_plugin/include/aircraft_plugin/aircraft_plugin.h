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
 *   to OSRF foundation providing 3D model and Cessna Plugins for gazebo 6
 */

#ifndef AICRAFT_PLUGIN_H
#define AICRAFT_PLUGIN_H


//Standard includes
#include <stdio.h>
#include <array>
#include <string>

// ROS messages
#include <std_msgs/Float32.h>
#include <mav_msgs/CommandMotorSpeed.h>

// This plugin implements a thread, based on boost, for the communication with Ardupilot
#include <boost/bind.hpp>
#include <mutex>

// Gazebo includes
#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

// ROS includes
#include <ros/callback_queue.h>
#include <ros/ros.h>

#define PI      3.1415926536

namespace gazebo {

   class AircraftPlugin : public ModelPlugin {

      // Constructor
      public: 
         AircraftPlugin();
         virtual ~AircraftPlugin();
         virtual void InitializeParams(); 

      protected:
         virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
         // GAZEBO related methods ------------------
         virtual void OnUpdate(const common::UpdateInfo & /*_info*/);
         virtual void OnControl(const mav_msgs::CommandMotorSpeedConstPtr& roll_velocities);

      private: 
         void UpdatePIDs(double _dt); 

         // Read an SDF parameter with a joint name and initialize a pointer to this joint.
            // _sdfParam SDF parameter containing a joint name.
            // _sdf Pointer to the SDF element containing the parameters.
            //_joint Pointer to the joint to be initialized.
            // return True if the SDF parameter is found and the joint name is found,
            // return  False otherwise.
         bool FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::JointPtr &_joint);

         /// Joint indexes.
         static const unsigned int Aileron  = 0;
         static const unsigned int Elevators    = 1;
         static const unsigned int Propeller    = 2;
         static const unsigned int Rudder       = 3;
         static const unsigned int Flap     = 4;

         static const unsigned int kLeftAileron  = 0;
         static const unsigned int kLeftFlap     = 1;
         static const unsigned int kRightAileron = 2;
         static const unsigned int kRightFlap    = 3;
         static const unsigned int kElevators    = 4;
         static const unsigned int kRudder       = 5;
         static const unsigned int kPropeller    = 6;

         // ROS messages
         std::string command_sub_topic_;
         std::string namespace_;
         ros::NodeHandle* node_handle_;
         ros::Subscriber command_sub_;

         // Gazebo messages / data
         physics::ModelPtr model_;
         physics::LinkPtr link_Propeller;
         event::ConnectionPtr updateConnection_;
         std::array<physics::JointPtr, 7> joints;

         // Timing
         gazebo::common::Time lastControllerUpdateTime; // keep track of controller update sim-time.
         boost::thread callback_queue_thread_;
         double sampling_time_ =0;
         double prev_sim_time_ = 0;
      
         // Max propeller RPM.
         int32_t propellerMaxRpm = 37500;
         double limit_Upper = 0;
         double limit_Lower = 0;	

         //Next command to be applied to the propeller and control surfaces.
         std::array<float, 5> cmds;
      
         // Velocity PID for the propeller.
         common::PID propellerPID;
      
         // Position PID for the control surfaces.
         std::array<common::PID, 6> controlSurfacesPID;         

         // Controller update mutex.
         std::mutex mutex;

         void QueueThread();
   };
}
#endif // AICRAFT_PLUGIN_H
