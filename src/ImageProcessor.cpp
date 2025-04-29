#include "ImageProcessor.hpp"

ImageProcessor::ImageProcessor(): Node("image_processor"), mpIt(nullptr), mLeftMapsInitialized(false) {
    using std::placeholders::_1;
    mpIt = new image_transport::ImageTransport((std::shared_ptr<rclcpp::Node>)this);
    mLeftSub = mpIt->subscribe("/cam_rgb_right", 10, std::bind(&ImageProcessor::LeftCallback, this, _1));
    mRightSub = mpIt->subscribe("/cam_rgb_left", 10, std::bind(&ImageProcessor::RightCallback, this, _1));
    mLeftSubInfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/cam_rgb_left_info", 10,
        std::bind(&ImageProcessor::CameraInfoCallbackL, this, std::placeholders::_1));
  
        mRightSubInfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/cam_rgb_right_info", 10,
        std::bind(&ImageProcessor::CameraInfoCallbackR, this, std::placeholders::_1));
    mLeftPub = mpIt->advertise("cam_rgb_left_post", QUEUE_SIZE);
    mRightPub = mpIt->advertise("cam_rgb_right_post", QUEUE_SIZE);
    RCLCPP_INFO(this->get_logger(), "Image processor node started.");
}

ImageProcessor::~ImageProcessor() {
    delete mpIt;
}

void ImageProcessor::LeftCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if(!mLeftMapsInitialized) return;

    cv::Mat raw = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat rectified;
    cv::remap(raw, rectified, mMap1Left, mMap2Left, cv::INTER_LINEAR);
    //sensor_msgs::msg::Image::SharedPtr newMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rectified).toImageMsg();
    //mLeftPub.publish(newMsg);
    //mLeftTempImage = ConvertToMat(rectified);
    mLeftImage = rectified.clone();
    ComputeDisparity();
}

void ImageProcessor::RightCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    if(!mRightMapsInitialized) return;
    cv::Mat raw = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat rectified;
    cv::remap(raw, rectified, mMap1Right, mMap2Right, cv::INTER_LINEAR);
    //sensor_msgs::msg::Image::SharedPtr newMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rectified).toImageMsg();
    //mRightPub.publish(newMsg);
    //mRightTempImage = ConvertToMat(rectified);
    mRightImage = rectified.clone();
    ComputeDisparity();
}

cv::Mat ImageProcessor::ConvertToMat(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    try {
        return cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

void ImageProcessor::ComputeDisparity() {
    //RCLCPP_INFO(this->get_logger(), "Computing");
    if (mLeftImage.empty() || mRightImage.empty()) return;
    //RCLCPP_INFO(this->get_logger(), "In image tested");
    cv::Mat grayL_cpu, grayR_cpu;
    cv::cuda::GpuMat grayL_gpu, grayR_gpu;
    cv::cvtColor(mLeftImage, grayL_cpu, cv::COLOR_BGR2GRAY);
    cv::cvtColor(mRightImage, grayR_cpu, cv::COLOR_BGR2GRAY);
    grayL_gpu.upload(grayL_cpu);
    grayR_gpu.upload(grayR_cpu);

    //RCLCPP_INFO(this->get_logger(), "Applying clahe");
    cv::Ptr<cv::CLAHE> clahe = cv::cuda::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(grayL_gpu, grayL_gpu);
    clahe->apply(grayR_gpu, grayR_gpu);
    // grayL = cv::Mat(grayL_gpu);
    // cv::imshow("clahe", grayL);
    int numDisparities = 16 * 5;
    int blockSize = 5;
    auto pSgbm_gpu = cv::cuda::StereoSGM::create(
        0, numDisparities, blockSize,
        8 * blockSize * blockSize,
        32 * blockSize * blockSize,
        1, 10, 100, 32, cv::StereoSGBM::MODE_SGBM
    );

    cv::Mat disparity_cpu;
    grayL_gpu.download(grayL_cpu);
    grayR_gpu.download(grayR_cpu);
    pSgbm_gpu->compute(grayL_cpu, grayR_cpu, disparity_cpu);
    disparity_cpu.convertTo(disparity_cpu, CV_32F, 1.0 / 16.0);

    cv::Mat disparity(disparity_cpu);
    cv::Mat dispVis;
    cv::normalize(disparity, dispVis, 0, 255, cv::NORM_MINMAX);
    dispVis.convertTo(dispVis, CV_8U);
    cv::imshow("Disparity", dispVis);
    cv::waitKey(1);

    mLeftImage = cv::Mat();
    mRightImage = cv::Mat();
}

void ImageProcessor::CameraInfoCallbackL(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (mLeftMapsInitialized) return;

    cv::Mat K = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    cv::Mat D = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
    cv::Size imageSize(msg->width, msg->height);

    cv::Mat R = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->r.data())).clone();
    cv::Mat P = cv::Mat(3, 4, CV_64F, const_cast<double*>(msg->p.data())).clone();

    // Compute remap matrices
    cv::initUndistortRectifyMap(K, D, R, P(cv::Rect(0, 0, 3, 3)), imageSize, CV_32FC1, mMap1Left, mMap2Left);

    mLeftMapsInitialized = true;
    RCLCPP_INFO(this->get_logger(), "LInfo loaded");
}

void ImageProcessor::CameraInfoCallbackR(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (mRightMapsInitialized) return;

    cv::Mat K = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    cv::Mat D = cv::Mat(msg->d.size(), 1, CV_64F, const_cast<double*>(msg->d.data())).clone();
    cv::Size imageSize(msg->width, msg->height);

    cv::Mat R = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->r.data())).clone();
    cv::Mat P = cv::Mat(3, 4, CV_64F, const_cast<double*>(msg->p.data())).clone();

    // Compute remap matrices
    cv::initUndistortRectifyMap(K, D, R, P(cv::Rect(0, 0, 3, 3)), imageSize, CV_32FC1, mMap1Right, mMap2Right);

    mRightMapsInitialized = true;
    RCLCPP_INFO(this->get_logger(), "RInfo loaded");
}