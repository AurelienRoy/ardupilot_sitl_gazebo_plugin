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
#include <math.h>



namespace gazebo
{
void ArdupilotSitlGazeboPlugin::check_parachute_cmd(float servo_parachute)
{
    // 0.5 is the threshold for parachute release
    if (servo_parachute < 0.5)
        return;
    
    if (_is_parachute_available) {
        ROS_INFO( PLUGIN_LOG_PREPEND "Releasing the parachute !!!");
        load_parachute_model();
        _is_parachute_available = false;   
    }
}


// METHOD 1: Add a new parachute model, and make a joint between the 2 models

void ArdupilotSitlGazeboPlugin::load_parachute_model()
{
    // Gets a pointer to the UAV model
    _uav_model = _parent_world->GetModel(_modelName);
    if (!_uav_model) {
        ROS_INFO( PLUGIN_LOG_PREPEND "UAV MODEL NOT FOUND !!!");
        return;
    } else {
        ROS_DEBUG( PLUGIN_LOG_PREPEND "UAV model pointer acquired");
    }
    
    // Creates the parachute model
    _parent_world->InsertModelFile("model://" PARACHUTE_MODEL_NAME);
    
    // Takes the lock, with an expiration date.
    // This way the simulation is paused until the model is fully loaded
    take_lapseLock(MAX_LAPSE_LOCK_ON_MODEL_INSERT);
    ROS_INFO( PLUGIN_LOG_PREPEND "Simulation shortly paused to load the parachute model");
}

    
/*
  Finishes the apparition of the parachute, by joining it to the UAV.
  
  Gazebo loads models asyncrhonously, in a thread. This callback method is called once
  the parachute model is loaded.
 */
void ArdupilotSitlGazeboPlugin::on_parachute_model_loaded()
{
    // This method is executed independently from the main loop thread.
    // Beware of access to shared variables memory. Use mutexes if required.
    
    _parachute_model = _parent_world->GetModel(PARACHUTE_MODEL_NAME);
    if (!_parachute_model) {
        ROS_ERROR( PLUGIN_LOG_PREPEND "Error: Parachute model failed to load !");
        // Frees the lock
        _loop_lapseLock = 0.0f;
        return;
    }

    const math::Pose uavPose = _uav_model->GetWorldPose();
    _parachute_model->SetWorldPose(math::Pose(uavPose.pos.x, uavPose.pos.y, uavPose.pos.z, 0, PI_2, 0));        // or use uavPose.ros.GetYaw() ?
    ROS_DEBUG( PLUGIN_LOG_PREPEND "Parachute pose: %f, %f, %f", uavPose.pos.x, uavPose.pos.y, uavPose.pos.z);

    // create the joint for the given model
    
    _uav_chute_joint = _parent_world->GetPhysicsEngine()->CreateJoint("ball", _uav_model);
    _uav_chute_joint->SetName("uav_chute_joint");
    ROS_DEBUG( PLUGIN_LOG_PREPEND "Joint created id %d", _uav_chute_joint->GetId()); 
    
    // retrieves links
    gazebo::physics::LinkPtr uav_link = _uav_model->GetLink(UAV_MODEL_CG_LINK);
    gazebo::physics::LinkPtr chute_link = _parachute_model->GetLink(PARACHUTE_MODEL_ATTACH_LINK);

    // attach joint to links
    _uav_chute_joint->Attach(uav_link, chute_link);

    // load the joint, and set up its anchor point
    _uav_chute_joint->Load(uav_link, chute_link, math::Pose(0, 0, 0, 0, 0, 0));

    // set the axis of revolution
    //_uav_chute_joint->SetAxis(0, math::Vector3(0,0,1));

    // set other joint attributes
    _uav_chute_joint->SetLowerLimit(0, math::Angle(-3));
    _uav_chute_joint->SetUpperLimit(0, math::Angle(3));
    _uav_chute_joint->SetDamping(0, 15);

    if (release_lapseLock())
        ROS_INFO( PLUGIN_LOG_PREPEND "Parachute model loaded, resuming the simulation"); 
    else
        ROS_INFO( PLUGIN_LOG_PREPEND "Parachute model loaded");
}

} // end of "namespace gazebo"



