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

#ifndef RECORDER_CAMERA_PLUGIN_H
#define RECORDER_CAMERA_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// Ardupilot plugin services
#include <ardupilot_sitl_gazebo_plugin/TakeApmLapseLock.h>
#include <ardupilot_sitl_gazebo_plugin/ReleaseApmLapseLock.h>

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write
#include <opencv2/imgproc/imgproc.hpp>

#include <string>



namespace gazebo
{
    class GazeboRecorderCamera : public SensorPlugin
    {  
      public:
        // Constructor
        public: GazeboRecorderCamera();

        // Destructor
        ~GazeboRecorderCamera();

        // Load the plugin
        // takeS in sensor parent and SDF root element
        public: void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);
        
      protected:
        
        // Called on every new camera frame (at the specified camera update rate, in sim time)
        virtual void OnNewFrame(const unsigned char *image,
                                unsigned int width, unsigned int height,
                                unsigned int depth, const std::string &format);

      private:
        
        //------------------------------------------------
        // Video/Frames recording methods
        
        bool ReadVideoCodec(const std::string &codec, int &cv_ext);
        
        // Returns true if successfull
        bool OpenOutputVideoFile();
        void ReleaseVideo();
        
        std::string GenerateNewFrameFilepath();
        std::string GenerateNewVideoFilepath();
        
        //------------------------------------------------
        // ROS image topic publishing methods
        
        // Returns true if successfull
        bool ReadEncodingFormat();
        
        bool fillImage(sensor_msgs::Image& image,
                        const std::string& encoding_arg,
                        uint32_t rows_arg,
                        uint32_t cols_arg,
                        uint32_t step_arg,
                        const void* data_arg);
        
        void PublishCameraData(const unsigned char *_image,
                                unsigned int _width, unsigned int _height,
                                unsigned int _depth, const std::string &_format);
        
        
        //------------------------------------------------
        // Member variables
        
        // Pointer to the update event connection
        ros::NodeHandle         _nh;
        event::ConnectionPtr    _newFrameConnection;
        
        // Pointers to Gazebo's parent sensor
        sensors::CameraSensorPtr    _parentSensor;
        rendering::CameraPtr        _camera;

        // Image parameters
        unsigned int _width;
        unsigned int _height;
        unsigned int _depth;
        
        // For Ardupilot's lapse-lock mechanism
        ros::ServiceClient          _client_take_lapseLock;
        ros::ServiceClient          _client_release_lapseLock;
        
        // For image transport
        image_transport::Publisher        _image_pub;
        image_transport::ImageTransport*  _itnode;
        sensor_msgs::Image                _image_msg;
        boost::mutex                      _image_lock;
        
        // ROS image topic publisher XACRO parameters
        std::string          _image_topic_name;
        std::string          _camera_name;
        
        // image encoding types
        std::string          _type;
        int                  _skip;
        
        // OpenCV variables
        cv::VideoWriter*     _outputVideo;
        
        // Video recorder XACRO parameters
        bool                 _doSaveVideo;
        std::string          _videoDir;
        std::string          _videoFilename;
        std::string          _videoFileExt;
        int                  _videoCodecExt;
        
        // Frames recorder XACRO parameters
        bool                 _doSaveFrames;
        std::string          _framesDir;
        std::string          _framesFilename;
        std::string          _framesFileExt;
        
        // Additional frames recording variables
        int                  _nbFramesCaptured;
        
    };
}
#endif  // RECORDER_CAMERA_PLUGIN_H
