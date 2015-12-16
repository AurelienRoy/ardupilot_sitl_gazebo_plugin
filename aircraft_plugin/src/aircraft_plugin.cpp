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
 */

#include "../include/aircraft_plugin/aircraft_plugin.h"


namespace gazebo {

   AircraftPlugin::AircraftPlugin() : cmds {{0, 0, 0, 0, 0}}{}
   
   AircraftPlugin::~AircraftPlugin() {
      event::Events::DisconnectWorldUpdateBegin(updateConnection_);
      if (node_handle_) {
          node_handle_->shutdown();
          delete node_handle_;
      }
      for (auto &pid : this->controlSurfacesPID)
      {
          pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);
          pid.SetCmd(0.0);
          pid.SetPGain(2000);
          pid.SetIGain(0);
          pid.SetDGain(0);
     }
   }

   void AircraftPlugin::InitializeParams() {}

   bool AircraftPlugin::FindJoint(const std::string &_sdfParam, sdf::ElementPtr _sdf, physics::JointPtr &_joint)
   {
      // Read the required plugin parameters.
      if (!_sdf->HasElement(_sdfParam))
      {
         gzerr << "Unable to find the <" << _sdfParam << "> parameter." << std::endl;
         return false;
      }

      std::string jointName = _sdf->Get<std::string>(_sdfParam);
      _joint = this->model_->GetJoint(jointName);
      if (!_joint)
      {
         gzerr << "Failed to find joint [" << jointName
             << "] aborting plugin load." << std::endl;
         return false;
      }
      return true;
   }

   void AircraftPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
      this->model_ = _model;

      namespace_.clear();

      if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
      else
      gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";

      ROS_INFO("AIRCRAFT: robotNamespace %s",namespace_.c_str());
      node_handle_ = new ros::NodeHandle(namespace_);

      if (_sdf->HasElement("propeller_max_rpm"))
      {
         this->propellerMaxRpm = _sdf->Get<int32_t>("propeller_max_rpm");
         if (this->propellerMaxRpm == 0)
         {
            this->propellerMaxRpm=2500;
         }
      }
      
      // Read the required joint name parameters.
      std::vector<std::string> requiredParams = {"left_aileron", "left_flap",
      "right_aileron", "right_flap", "elevators", "rudder", "propeller"};

      for (size_t i = 0; i < requiredParams.size(); ++i)
      {
         if (!this->FindJoint(requiredParams[i], _sdf, this->joints[i]));
         //return;
      }

      if (_sdf->HasElement("commandSubTopic"))
      {
         this->command_sub_topic_ =  _sdf->GetElement("commandSubTopic")->Get<std::string>();
      } 
      if (_sdf->HasElement("propeller_link"))
      {
         std::string link_Propeller_str=  _sdf->GetElement("propeller_link")->Get<std::string>();
         this->link_Propeller=this->model_->GetLink(link_Propeller_str);
      } 

      this->lastControllerUpdateTime = this->model_->GetWorld()->GetSimTime();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&AircraftPlugin::OnUpdate, this, _1));

      this->command_sub_ = node_handle_->subscribe(this->command_sub_topic_, 1000, &AircraftPlugin::OnControl, this);
      this->limit_Upper = this->joints[kLeftAileron]->GetUpperLimit(0).Degree() ;
      this->limit_Lower = this->joints[kLeftFlap]->GetLowerLimit(0).Degree() ;
      this->joints[kLeftAileron]->SetAngle(0, 0);
      this->joints[kRightAileron]->SetAngle(0, 0);
      this->joints[kElevators]->SetAngle(0,0);
      this->joints[kRudder]->SetAngle(0, 0);
      this->joints[kLeftFlap]->SetAngle(0, 0);
      this->joints[kRightFlap]->SetAngle(0, 0);
   }

   // This gets called by the world update start event.
   void AircraftPlugin::OnUpdate(const common::UpdateInfo& _info) {
      
      std::lock_guard<std::mutex> lock(this->mutex);
      gazebo::common::Time curTime = this->model_->GetWorld()->GetSimTime();

      sampling_time_ = _info.simTime.Double() - prev_sim_time_;
      prev_sim_time_ = _info.simTime.Double();
      if (curTime > this->lastControllerUpdateTime)
      {
         // Update the control surfaces and publish the new state.
         this->UpdatePIDs((curTime - this->lastControllerUpdateTime).Double());
         this->lastControllerUpdateTime = curTime;
      }
   }
   
   void AircraftPlugin::UpdatePIDs(double _dt)
   {  
      // Position PID for the control surfaces.
      float Aileron_val = -1*((this->cmds[Aileron]-500)*(limit_Upper/500)*PI/180);
      float Elevators_val = -1*((this->cmds[Elevators]-500)*(limit_Upper/500)*PI/180);
      float Rudder_val = ((this->cmds[Rudder]-500)*(limit_Upper/500)*PI/180);
      float throttle_val = ((this->cmds[Propeller])/1000.0);
      float Flap_val = ((this->cmds[Flap])*(limit_Lower/1000)*PI/180);

      double pos = this->joints[kElevators]->GetAngle(0).Radian();
      double error = pos - Elevators_val;
      this->joints[kElevators]->SetForce(0, error*-5);

      pos = this->joints[kLeftAileron]->GetAngle(0).Radian();
      error = pos - Aileron_val;
      this->joints[kLeftAileron]->SetForce(0, error*-5);

      pos = this->joints[kRightAileron]->GetAngle(0).Radian();
      error = pos - (-1*Aileron_val);
      this->joints[kRightAileron]->SetForce(0, error*-5);

      pos = this->joints[kRudder]->GetAngle(0).Radian();
      error = pos - Rudder_val;
      this->joints[kRudder]->SetForce(0, error*-5);

      pos = this->joints[kLeftFlap]->GetAngle(0).Radian();
      error = pos - Flap_val;
      this->joints[kLeftFlap]->SetForce(0, error*-5);
      pos = this->joints[kRightFlap]->GetAngle(0).Radian();
      error = pos - Flap_val;
      this->joints[kRightFlap]->SetForce(0, error*-5);

      if(throttle_val>0)
      {
         double vel = this->joints[kPropeller]->GetVelocity(0);
         double maxVel = this->propellerMaxRpm*2.0*M_PI/60.0;
         double target = maxVel * throttle_val;
         error = vel - target;
         this->joints[kPropeller]->SetForce(0, error*-1000);
         //ROS_WARN("%f",error);
      }
      else{
         //ROS_WARN("throttle_val=<0 %f",throttle_val);
      }
   }

   void AircraftPlugin::OnControl(const mav_msgs::CommandMotorSpeedConstPtr& roll_velocities) {
      this->cmds[Aileron] = roll_velocities->motor_speed[Aileron];
      this->cmds[Elevators] = roll_velocities->motor_speed[Elevators];
      this->cmds[Propeller] = roll_velocities->motor_speed[Propeller]/10;
      this->cmds[Rudder] = roll_velocities->motor_speed[Rudder];
      this->cmds[Flap] = roll_velocities->motor_speed[Flap];
   }

   GZ_REGISTER_MODEL_PLUGIN(AircraftPlugin);
}
