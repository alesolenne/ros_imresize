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
_undistord(),
saveCameraInfo(),
imgTopicName(),
infoTopicName(),
fps(0),
desired_path(),
_it(_nh)
{

    ros::param::get("~/resize_width", _width);
    ros::param::get("~/resize_height", _height);
    ros::param::get("~/image_crop", imgTopicName);
    ros::param::get("~/info_crop", infoTopicName);
    ros::param::get("~/save_camera_info", saveCameraInfo);
    ros::param::get("~/fps", fps);
    ros::param::get("~/desired_path", desired_path);
    ros::param::get("~/undistord", _undistord);


    _sub_info = _nh.subscribe(infoTopicName, 1, &SingleImageHandler::setCameraInfo, this);

    _sub_img = _it.subscribe(imgTopicName, 1, &SingleImageHandler::topicCallback, this);

    _pub_img = _it.advertise(imgTopicName + "_crop", 1);

    _pub_info = _nh.advertise<sensor_msgs::CameraInfo>(infoTopicName + "_crop", 1);
    
    ros::Rate wrait(fps);

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
 
    if (_undistord)
    {
       cv::undistort(cvPtr->image, undist, _K, _dist);
    }
    else
    {
       undist = cvPtr->image;
    }   

    undist = cvPtr->image;
    
    cv::resize(undist, cvPtr->image, cv::Size(_width, _height),
               0, 0, cv::INTER_LINEAR);

    _pub_img.publish(cvPtr->toImageMsg());
    _pub_info.publish(_infoCam);

    char username[32];
    cuserid(username);
    std::string username_str(username);

    if (saveCameraInfo)
    {

        ofstream output_file("/home/" + username_str + desired_path);

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
        
        saveCameraInfo = false;
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

    if (_undistord)
    {
        _K = cv::Mat::eye(3, 3, CV_32F);

        _K.at<float>(0) = _infoCam.K[0];
        _K.at<float>(2) = _infoCam.K[2];

        _K.at<float>(4) = _infoCam.K[4];
        _K.at<float>(5) = _infoCam.K[5];

        _dist = cv::Mat(_infoCam.D);
    }

    ROS_INFO_STREAM_ONCE("Resize node is running for camera info topic!");
}
