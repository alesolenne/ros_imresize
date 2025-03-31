#include "ros_imresize/image_handler.h"

using nlohmann::json;
using namespace std;

SingleImageHandler::SingleImageHandler()
    : nh("/ros_imresize"),
      width(0),
      height(0),
      undistort(false),
      resize(false),
      saveCameraInfo(false),
      fps(0),
      it(nh) {
    // Fetch parameters with error handling
    if (!ros::param::get("~/resize_width", width) ||
        !ros::param::get("~/resize_height", height) ||
        !ros::param::get("~/image_topic", imgTopicName) ||
        !ros::param::get("~/info_topic", infoTopicName) ||
        !ros::param::get("~/save_camera_info", saveCameraInfo) ||
        !ros::param::get("~/fps", fps) ||
        !ros::param::get("~/desired_path", desired_path) ||
        !ros::param::get("~/undistort", undistort) ||
        !ros::param::get("~/convert_encoding", convert_encoding) ||
        !ros::param::get("~/resize", resize)) {
        ROS_ERROR("Failed to fetch one or more parameters. Shutting down.");
        ros::shutdown();
        return;
    }

    sub_info = nh.subscribe(infoTopicName, 1, &SingleImageHandler::setCameraInfo, this);
    sub_img = it.subscribe(imgTopicName, 1, &SingleImageHandler::setImage, this);

    // Modify topic names based on settings
    imgTopicName.erase(imgTopicName.end() - 3, imgTopicName.end()); // Remove "raw char"
    string new_image_topic_name, new_info_topic_name;

    if (undistort && resize) {
        new_image_topic_name = imgTopicName + "rect_crop";
        new_info_topic_name = infoTopicName + "_crop";
        ROS_INFO("Undistortion and resize are set!");
    } else if (!undistort && resize) {
        new_image_topic_name = imgTopicName + "raw_crop";
        new_info_topic_name = infoTopicName + "_crop";
        ROS_INFO("Resize is set!");
    } else if (undistort && !resize) {
        new_image_topic_name = imgTopicName + "rect";
        ROS_INFO("Undistortion is set!");
    } else {
        ROS_WARN("No resize or undistortion is set! Shutting down.");
        ros::shutdown();
        return;
    }

    pub_img = it.advertise(new_image_topic_name, 1);
    if (resize) {
        pub_info = nh.advertise<sensor_msgs::CameraInfo>(new_info_topic_name, 1);
    }
    
    ROS_INFO("Waiting for camera topics...");
}

SingleImageHandler::~SingleImageHandler() {
    ROS_INFO("Shutting down node.");
}

void SingleImageHandler::setImage(const sensor_msgs::ImageConstPtr& received_image) {
    
    cv_bridge::CvImagePtr cvPtr;

    try {
        ROS_INFO_STREAM_ONCE("Image received!");

        if (convert_encoding)
        {
            cvPtr = cv_bridge::toCvCopy(received_image, sensor_msgs::image_encodings::RGB8);
        }
        else
        {
            cvPtr = cv_bridge::toCvCopy(received_image, received_image->encoding);
        }

        cv::Mat processed_image;

        if (undistort) {
            cv::undistort(cvPtr->image, processed_image, K, dist);
        } else {
            processed_image = cvPtr->image;
        }

        if (resize) {
            cv::resize(processed_image, cvPtr->image, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
            pub_info.publish(infoCam);
        }

        pub_img.publish(cvPtr->toImageMsg());
        ROS_INFO_STREAM_ONCE("Image and Info processed!");

        if (saveCameraInfo) {

            char username[32];
            cuserid(username);
            string username_str(username);

            if (!username) {
                ROS_ERROR("Failed to get username. Skipping camera info save.");
                return;
            }

            ofstream output_file("/home/" + string(username) + desired_path);
            if (!output_file.is_open()) {
                ROS_ERROR("Failed to open file for saving camera info.");
                return;
            }

            json outJson;
            outJson["K"] = vector<double>(infoCam.K.begin(), infoCam.K.end());
            outJson["P"] = vector<double>(infoCam.P.begin(), infoCam.P.end());
            outJson["h"] = infoCam.height;
            outJson["w"] = infoCam.width;

            output_file << outJson.dump(4);
            output_file.close();

            saveCameraInfo = false;
            ROS_INFO("Saved new camera calibration to camera.json file.");
        }
    } catch (const exception& e) {
        ROS_ERROR("Error in setImage: %s", e.what());
    }
}

void SingleImageHandler::setCameraInfo(const sensor_msgs::CameraInfoConstPtr& received_info) {
    try {

        infoCam = *received_info;

        if (undistort) {
            K = cv::Mat::eye(3, 3, CV_32F);
            K.at<float>(0, 0) = infoCam.K[0];
            K.at<float>(0, 2) = infoCam.K[2];
            K.at<float>(1, 1) = infoCam.K[4];
            K.at<float>(1, 2) = infoCam.K[5];
            dist = cv::Mat(infoCam.D);
        }

        if (resize) {
            float scale_x = static_cast<float>(width) / infoCam.width;
            float scale_y = static_cast<float>(height) / infoCam.height;

            infoCam.K[0] *= scale_x;
            infoCam.K[2] *= scale_x;
            infoCam.K[4] *= scale_y;
            infoCam.K[5] *= scale_y;

            infoCam.P[0] *= scale_x;
            infoCam.P[2] *= scale_x;
            infoCam.P[5] *= scale_y;
            infoCam.P[6] *= scale_y;

            infoCam.width = width;
            infoCam.height = height;
        }

        ROS_INFO_STREAM_ONCE("Camera info received!");
    } catch (const exception& e) {
        ROS_ERROR("Error in setCameraInfo: %s", e.what());
    }
}