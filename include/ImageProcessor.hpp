/**
 * @file ImageProcessor.hpp
 * @author tlr
 * @brief The class of a node that takes image from a topic, modify them and send them onto another topic
 * @version 0.0.0
 * @date 2025-04-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

// Lib includes
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/cudastereo.hpp>
#include <opencv4/opencv2/cudaimgproc.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// ROS2 queue size
constexpr uint8_t QUEUE_SIZE = 1;


class ImageProcessor : public rclcpp::Node {
private:
    std::shared_ptr<rclcpp::Node> mNode;
    image_transport::ImageTransport* mpIt;
    image_transport::Subscriber mLeftSub, mRightSub;
    image_transport::Publisher mLeftPub, mRightPub;
    cv::Mat mLeftImage, mRightImage;
    cv::Mat mLeftTempImage, mRightTempImage;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr mLeftSubInfo, mRightSubInfo;
    cv::Mat mMap1Left, mMap2Left;
    bool mLeftMapsInitialized;
    cv::Mat mMap1Right, mMap2Right;
    bool mRightMapsInitialized;
public:
    ImageProcessor();
    ~ImageProcessor();
    void CameraInfoCallbackL(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void CameraInfoCallbackR(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void LeftCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void RightCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    cv::Mat ConvertToMat(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void ComputeDisparity();
};