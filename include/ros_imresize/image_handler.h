#ifndef IMAGE_HANDLER_H_
#define IMAGE_HANDLER_H_

// OpenCV headers
#include <opencv2/opencv.hpp>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// Other libraries
#include <cv_bridge/cv_bridge.h>
#include <nlohmann/json.hpp>

class SingleImageHandler
{
public:
    // Constructor and Destructor
    SingleImageHandler();
    ~SingleImageHandler();

    // Check if an image has been received
    bool isImageReceived() const;

protected:
    // Callback to set camera info
    void setCameraInfo(const sensor_msgs::CameraInfoConstPtr& camera_info_msg);

    // Callback for image topic
    void topicCallback(const sensor_msgs::ImageConstPtr& received_image);

    // Camera info
    sensor_msgs::CameraInfo infoCam;

    // ROS NodeHandle and ImageTransport
    ros::NodeHandle nh;

    image_transport::ImageTransport it;

    // Subscribers and Publishers
    image_transport::Subscriber sub_img;
    image_transport::Publisher pub_img;

    ros::Publisher pub_info;
    ros::Subscriber sub_info;

    // Camera parameters
    cv::Mat K;  // Intrinsic camera matrix
    cv::Mat dist;    // Distortion coefficients

    // Flags
    bool saveCameraInfo;
    bool undistort;
    bool resize;

    // Image resizing parameters
    int width;
    int height;
    int fps;

    // Topic and file path names
    std::string imgTopicName;
    std::string infoTopicName;
    std::string desired_path;
};


#endif  // IMAGE_HANDLER_H_
