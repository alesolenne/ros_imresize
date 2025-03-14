#include "ros_imresize/image_handler.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/callback_queue.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <fstream>
#include <nlohmann/json.hpp>

using nlohmann::json;
using namespace std;

SingleImageHandler::SingleImageHandler() :
_nh("/ros_imresize"),
_width(0),
_height(0),
_it(_nh)
{
    string imgTopicName;
    string infoTopicName;
    bool set_img = false;

    ros::param::get("resize_width", _width);
    ros::param::get("resize_height", _height);
    ros::param::get("topic_crop", imgTopicName);
    ros::param::get("camera_info", infoTopicName);

    ros::Subscriber sub_info = _nh.subscribe(infoTopicName, 1, &SingleImageHandler::setCameraInfo, this);

    _sub_img = _it.subscribe(imgTopicName, 1, &SingleImageHandler::topicCallback, this);

    _pub_img = _it.advertise(imgTopicName + "_crop", 1);

    _pub_info = _nh.advertise<sensor_msgs::CameraInfo>(infoTopicName + "_crop", 1);
    
    ros::Rate wrait(100);

    ROS_INFO("Waiting for camera topics ...");

    while(ros::ok())
    {
	ros::spinOnce();
	wrait.sleep();
    }
}

SingleImageHandler::~SingleImageHandler()
{
    ROS_INFO("Shutdown node");
    ros::shutdown();
}

void SingleImageHandler::topicCallback(const sensor_msgs::ImageConstPtr& received_image)
{
    cv_bridge::CvImagePtr cvPtr;
    const std::string encoding_image = received_image.get()->encoding;

    cvPtr = cv_bridge::toCvCopy(received_image, received_image.get()->encoding);
       
    cv::Mat undist;
    undist = cvPtr->image;
    cv::resize(undist, cvPtr->image, cv::Size(_width, _height),
               0, 0, cv::INTER_LINEAR);

    _pub_img.publish(cvPtr->toImageMsg());
    _pub_info.publish(_infoCam);

    if (!set_img)
    {
        ofstream output_file("/home/vispci/catkin_ws/src/visp_megapose/params/camera.json");

        json outJson;

        double K [3][3] = {{_infoCam.K[0], _infoCam.K[1], _infoCam.K[2]},
                           {_infoCam.K[3], _infoCam.K[4], _infoCam.K[5]},
                           {_infoCam.K[6], _infoCam.K[7], _infoCam.K[8]}};
        
        double P [3][4] = {{_infoCam.P[0], _infoCam.P[1], _infoCam.P[2], _infoCam.P[3]},
                           {_infoCam.P[4], _infoCam.P[5], _infoCam.P[6], _infoCam.P[7]},
                           {_infoCam.P[8], _infoCam.P[9], _infoCam.P[10], _infoCam.P[11]}};

        uint w = _infoCam.width;
        uint h = _infoCam.height;

        outJson["K"] = K;
        outJson["P"] = P;
        outJson["h"] = h;
        outJson["w"] = w;

        output_file << outJson.dump(4);
        output_file.close(); 
        
        set_img = true;
        ROS_INFO("Print new camera calibration on camera.json file!");

    }
}

void SingleImageHandler::setCameraInfo(const sensor_msgs::CameraInfoConstPtr &received_info)
{
    _infoCam = *received_info;

    float scale_x = (float)(_width) / (float)(_infoCam.width);
    float scale_y = (float)(_height) / (float)(_infoCam.height);

    _infoCam.K[0] *= scale_x;
    _infoCam.K[2] *= scale_x;

    _infoCam.K[4] *= scale_y;
    _infoCam.K[5] *= scale_y;
    
    _infoCam.P[0] *= scale_x;
    _infoCam.P[2] *= scale_x;

    _infoCam.P[5] *= scale_y;
    _infoCam.P[6] *= scale_y;

    _infoCam.width = _width;
    _infoCam.height = _height;

    // ROS_INFO_STREAM("Previous camera info :\n" << *received_info << "\n");
    // ROS_INFO_STREAM("New camera info :\n" << _infoCam << "\n");
    ROS_INFO_STREAM_ONCE("Resize node is running for camera info topic!");
}