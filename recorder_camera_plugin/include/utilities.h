/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Additional authors (small modifications):
 *   Aurelien ROY
 *   Maxime LAFLEUR
 */

#ifndef RECORDER_CAMERA_UTILITIES_H
#define RECORDER_CAMERA_UTILITIES_H

#include <gazebo/gazebo.hh>

// Log/Debug
#define PLUGIN_LOG_PREPEND       "REC_CAM: "


namespace gazebo
{

    /**
    * \brief Obtains a parameter from sdf.
    * \param[in] sdf Pointer to the sdf object.
    * \param[in] name Name of the parameter.
    * \param[out] param Param Variable to write the parameter to.
    * \param[in] default_value Default value, if the parameter not available.
    * \param[in] verbose If true, gzerror if the parameter is not available.
    */
    // From: rotors_simulator (ethz)
    template<class T>
    bool getSdfParam(sdf::ElementPtr sdf,
                     const std::string& name,
                     T& param,
                     const T& default_value,
                     const bool& verbose = false) {
        
        if (sdf->HasElement(name)) {
            param = sdf->GetElement(name)->Get<T>();
            return true;
        } else {
            param = default_value;
            if (verbose)
              gzerr << PLUGIN_LOG_PREPEND "Please specify a value for parameter \"" << name << "\".\n";
            }
            return false;
    }

} // end of namespace gazebo

#endif  // RECORDER_CAMERA_UTILITIES_H
