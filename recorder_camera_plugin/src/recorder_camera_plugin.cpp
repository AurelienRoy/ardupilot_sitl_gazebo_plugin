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

#include "../include/recorder_camera_plugin.h"
#include "../include/utilities.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

// Image convertions to OpenCV
#include <cv_bridge/cv_bridge.h>



namespace gazebo
{
    // Register this plugin with the simulator
    GZ_REGISTER_SENSOR_PLUGIN(GazeboRecorderCamera)
    
    // ------------------------------------------------------
    // Constructor
    GazeboRecorderCamera::GazeboRecorderCamera():
         SensorPlugin(),
        _nh("recorder_camera_plugin"),
        _width(0),
        _height(0),
        _depth(0),
        _outputVideo(NULL),
        _doSaveVideo(false),
        _doSaveFrames(false),
        _nbFramesCaptured(0)
    
    {
    }

    // ------------------------------------------------------
    // Destructor
    GazeboRecorderCamera::~GazeboRecorderCamera()
    {
        ReleaseVideo();
        
        ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
        this->_parentSensor.reset();
        this->_camera.reset();
    }
    

    void GazeboRecorderCamera::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
    {
        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }
        
        // Registers the parent camera sensor pointer
        if (!sensor)
            gzerr << "Invalid sensor pointer.\n";
        this->_parentSensor = boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
        if (!this->_parentSensor) {
            gzerr << PLUGIN_LOG_PREPEND "recorder_camera_plugin requires a CameraSensor.\n";
            return;
        }
    
        // Registers the attached camera
        this->_camera = this->_parentSensor->GetCamera();
        if (!this->_camera) {
            gzerr << PLUGIN_LOG_PREPEND "CameraPlugin not attached to a camera sensor\n";
            return;
        }
        
        getSdfParam<std::string>(sdf, "imageTopicName", _image_topic_name, "record_cam");
        getSdfParam<std::string>(sdf, "cameraName", _camera_name, "my_camera");
        
        // Video
        getSdfParam<bool>(sdf, "saveVideo",      _doSaveVideo,  false);
        if (_doSaveVideo) {
            getSdfParam<std::string>(sdf, "videoDir",      _videoDir,      "/tmp/");
            getSdfParam<std::string>(sdf, "videoFilename", _videoFilename, "video");
            getSdfParam<std::string>(sdf, "videoFileExt",  _videoFileExt,  "avi");
            
            std::string codecCodeName;
            getSdfParam<std::string>(sdf, "videoCodec",    codecCodeName,    "PIM1");   // PIM1 is MPEG-1
            ReadVideoCodec(codecCodeName, _videoCodecExt);
                        
            // Note:
            //   The destination folder must already exist. The program does not create it by itself.
        }
        
        // Frames
        getSdfParam<bool>(sdf, "saveFrames",      _doSaveFrames,  false);
        if (_doSaveFrames) {
            getSdfParam<std::string>(sdf, "framesDir",      _framesDir,      "/tmp/frames/");
            getSdfParam<std::string>(sdf, "framesFilename", _framesFilename, "frame");
            getSdfParam<std::string>(sdf, "framesFileExt",  _framesFileExt,  "jpg");
                        
            // Note:
            //   The destination folder must already exist. The program does not create it by itself.
        }
        
        this->_width  = this->_camera->GetImageWidth();
        this->_height = this->_camera->GetImageHeight();
        this->_depth  = this->_camera->GetImageDepth();
        
        // If you wish to get the world pointer, use these lines:
        //    std::string worldName = this->_parentSensor->GetWorldName();
        //    gazebo::physics::WorldPtr world = physics::get_world(worldName);
        //    common::Time last_update_time = this->_world->GetSimTime();
        
        ReadEncodingFormat();
        
        
        // Initializes the lapse-lock services of Ardupilot
        _client_take_lapseLock = this->_nh.serviceClient<ardupilot_sitl_gazebo_plugin::TakeApmLapseLock>("/fdmUDP/take_apm_lapseLock");
        _client_release_lapseLock = this->_nh.serviceClient<ardupilot_sitl_gazebo_plugin::ReleaseApmLapseLock>("/fdmUDP/release_apm_lapseLock");
        
        // Initializes the publisher of camera's frames
        this->_itnode    = new image_transport::ImageTransport(this->_nh);
        this->_image_pub = this->_itnode->advertise(this->_image_topic_name, 1);

        
        this->_newFrameConnection = this->_camera->ConnectNewImageFrame(
                             boost::bind(&GazeboRecorderCamera::OnNewFrame, this, _1, _2, _3, _4, _5));
        this->_parentSensor->SetActive(true);
        
        // Initializes the camera output video
        if (_doSaveVideo)
            OpenOutputVideoFile();
        
        std::ostringstream log_msg;
        log_msg << "";
        
        if (_doSaveFrames || _doSaveVideo) {
            log_msg << " and record ";
        }
        
        if (_doSaveFrames) {
            log_msg << "frames (" << _framesFilename << "0XXXX." << _framesFileExt << ")";
        }
         
        if (_doSaveVideo) {
            if (_doSaveFrames)
                log_msg << " and ";
            log_msg << "video (" << _videoFilename << "." << _videoFileExt << ")";
        }
        std::string log_msg_str;
        log_msg_str = log_msg.str();
        
        // Launch sumary
        ROS_INFO( PLUGIN_LOG_PREPEND "'%s' will publish %s at %.0f Hz",
                 this->_parentSensor->GetName().c_str(), log_msg_str.c_str(), this->_parentSensor->GetUpdateRate());
    }
    
    
    
    // ------------------------------------------------------
    //  Callbacks
    
    // On new frame is called at the specified camera update rate (in simulation time)
    void GazeboRecorderCamera::OnNewFrame(const unsigned char *image,
                                    unsigned int width, unsigned int height, unsigned int depth,
                                    const std::string &format)
    {
        // This method is called at the specified sensor update rate, using the simulation
        // time as time reference. (so no need for custom update rate system.
        
        // If video or frames are saved, pauses the simulation until the write is finished
        if (_doSaveFrames || _doSaveVideo) {
            ardupilot_sitl_gazebo_plugin::TakeApmLapseLock srv_data;
            srv_data.request.process_id = this->_parentSensor->GetName();
            srv_data.request.max_duration = 5;  // [s]

            if (!_client_take_lapseLock.call(srv_data)) {
                ROS_ERROR( PLUGIN_LOG_PREPEND "Failed to call service take_lapseLock, some frames may be lost");
            }
        }
        
        PublishCameraData(image, width, height, depth, format);
        
        // OpenCV image type
        cv_bridge::CvImagePtr cv_ptr;
        
        // Converts the Gazebo image to an OpenCV image
        try {
            cv_ptr = cv_bridge::toCvCopy(this->_image_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR( PLUGIN_LOG_PREPEND "cv_bridge exception: %s", e.what());
            return;
        }
        
        // If Frames recording is enabled, then generates a new frame filename, and saves the images
        if (_doSaveFrames) {
            std::string newFramepath;
            newFramepath = GazeboRecorderCamera::GenerateNewFrameFilepath();
            cv::imwrite(newFramepath, cv_ptr->image);
        }
        
        // Adds the frame to the video
        if (_doSaveVideo && (_outputVideo != NULL)) {
            if (_outputVideo->isOpened()) {
                static int nb_img = 0;
                this->_outputVideo->write(cv_ptr->image);
                boost::this_thread::sleep(boost::posix_time::seconds(0.1));
                
                nb_img++;
                ROS_DEBUG( PLUGIN_LOG_PREPEND "nb imgs saved = %d", nb_img);
            }
        }
        
        
        // If video or frames are saved, pauses the simulation until the write is finished
        if (_doSaveFrames || _doSaveVideo) {
            ardupilot_sitl_gazebo_plugin::ReleaseApmLapseLock srv_data2;
            srv_data2.request.process_id = this->_parentSensor->GetName();

            if (!_client_release_lapseLock.call(srv_data2)) {
                ROS_ERROR( PLUGIN_LOG_PREPEND "Failed to call service release_lapseLock, some frames may be lost");
            }
        }
    }
    
    
    
    // ------------------------------------------------------
    //  OpenCV Video Recording methods
    
    bool GazeboRecorderCamera::ReadVideoCodec(const std::string &codec, int &cv_ext)
    {
        if (codec.length() != 4) {
            ROS_ERROR( PLUGIN_LOG_PREPEND "Codec name codes should be 4 characters long, unlike '%s'\n", codec.c_str());
            // By default uses MPEG-1
            cv_ext = CV_FOURCC('P','I','M','1');
            return false;
        }
        
        cv_ext = CV_FOURCC(codec.at(0), codec.at(1), codec.at(2), codec.at(3));
    }
    
    
    bool GazeboRecorderCamera::OpenOutputVideoFile()
    {
        cv::Size video_resol;
        std::string filepath;
        double video_fps;
        
        // Sanity checks
        if ((!this->_parentSensor) || (this->_width == 0) || (this->_height == 0)) {
            ReleaseVideo();
            return false;
        }
        
        
        video_fps = this->_parentSensor->GetUpdateRate();
        video_resol = cv::Size(this->_width, this->_height);
        
        filepath = GenerateNewVideoFilepath();
        
        _outputVideo = new cv::VideoWriter();
        //_outputVideo->open(filepath, _videoCodecExt, video_fps/3, video_resol, true);
        //_outputVideo->open(filepath, _videoCodecExt, 30, video_resol, true);
        _outputVideo->open(filepath, _videoCodecExt, video_fps, video_resol, true);
        if (!_outputVideo->isOpened()) {
            ROS_ERROR( PLUGIN_LOG_PREPEND "Could not open the output video file '%s'\n", filepath.c_str());
            return false;
        }
        
        return true;
    }
    
    void GazeboRecorderCamera::ReleaseVideo()
    {
        if (_outputVideo != NULL) {
            ROS_INFO( PLUGIN_LOG_PREPEND "Closing the video file");
            _outputVideo->release();
            delete _outputVideo;
            _outputVideo = NULL;
        }
    }
    
    
    std::string GazeboRecorderCamera::GenerateNewFrameFilepath()
    {
        std::ostringstream stringStream;
        // Directory path + base filename
        stringStream << _framesDir << _framesFilename;
        // Frame's number
        stringStream << std::setfill('0') << std::setw(5) << _nbFramesCaptured;
        // File extension
        stringStream << "." << _framesFileExt;
        
        // Increments the total number of frames
        _nbFramesCaptured++;
        return stringStream.str();
    }
    

    
    std::string GazeboRecorderCamera::GenerateNewVideoFilepath()
    {
        std::ostringstream stringStream;
        
        // Form the new name with container
        stringStream << _videoDir << _videoFilename << "." << _videoFileExt;
        
        return stringStream.str();
    }
    

    // ------------------------------------------------------
    //  ROS image topic publishing methods
    
    // Partially based on the source code of: GazeboRosCameraUtils
    bool GazeboRecorderCamera::ReadEncodingFormat()
    {
        std::string format;
        
        // Sanity check
        if (!this->_camera) {
            this->_type = sensor_msgs::image_encodings::BGR8;
            this->_skip = 3;
            return false;
        }
        
        format = this->_camera->GetImageFormat();
        
        if (format == "L8") {
            this->_type = sensor_msgs::image_encodings::MONO8;
            this->_skip = 1;
        } else if (format == "R8G8B8") {
            this->_type = sensor_msgs::image_encodings::RGB8;
            this->_skip = 3;
        } else if (format == "B8G8R8") {
            this->_type = sensor_msgs::image_encodings::BGR8;
            this->_skip = 3;
        } else if (format == "BAYER_RGGB8") {
            ROS_INFO( PLUGIN_LOG_PREPEND "bayer simulation maybe computationally expensive.");
            this->_type = sensor_msgs::image_encodings::BAYER_RGGB8;
            this->_skip = 1;
        } else if (format == "BAYER_BGGR8") {
            ROS_INFO( PLUGIN_LOG_PREPEND "bayer simulation maybe computationally expensive.");
            this->_type = sensor_msgs::image_encodings::BAYER_BGGR8;
            this->_skip = 1;
        } else if (format == "BAYER_GBRG8") {
            ROS_INFO( PLUGIN_LOG_PREPEND "bayer simulation maybe computationally expensive.");
            this->_type = sensor_msgs::image_encodings::BAYER_GBRG8;
            this->_skip = 1;
        } else if (format == "BAYER_GRBG8") {
            ROS_INFO( PLUGIN_LOG_PREPEND "bayer simulation maybe computationally expensive.");
            this->_type = sensor_msgs::image_encodings::BAYER_GRBG8;
            this->_skip = 1;
        } else {
            ROS_ERROR( PLUGIN_LOG_PREPEND "Unsupported Gazebo ImageFormat\n");
            this->_type = sensor_msgs::image_encodings::BGR8;
            this->_skip = 3;
            return false;
        }
        return true;
    }
    
    
    // Partially based on the source code of: GazeboRosCameraUtils
    void GazeboRecorderCamera::PublishCameraData(const unsigned char *image,
                                    unsigned int width, unsigned int height, unsigned int depth,
                                    const std::string &format)
    {
        boost::mutex::scoped_lock lock(this->_image_lock);

        common::Time sensor_update_time;
        sensor_update_time = this->_parentSensor->GetLastUpdateTime();

        // copy data into image
        this->_image_msg.header.frame_id = this->_camera_name.c_str();
        this->_image_msg.header.stamp.sec = sensor_update_time.sec;
        this->_image_msg.header.stamp.nsec = sensor_update_time.nsec;

        // copy from src to image_msg_
        fillImage(this->_image_msg, this->_type, height, width, this->_skip * width,
                  reinterpret_cast<const void*>(image));

        // publish to ros
        this->_image_pub.publish(this->_image_msg);
    }
    
    // Partially based on the source code of: GazeboRosCameraUtils
    inline bool GazeboRecorderCamera::fillImage(sensor_msgs::Image& image,
                                                const std::string& encoding_arg,
                                                uint32_t rows_arg,
                                                uint32_t cols_arg,
                                                uint32_t step_arg,
                                                const void* data_arg)
    {
        image.encoding = encoding_arg;
        image.height   = rows_arg;
        image.width    = cols_arg;
        image.step     = step_arg;
        size_t st0 = (step_arg * rows_arg);
        image.data.resize(st0);
        memcpy(&image.data[0], data_arg, st0);
        image.is_bigendian = 0;
        return true;
    }
    
}
