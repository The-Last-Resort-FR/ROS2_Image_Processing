#include <cstdio>
#include "ImageProcessor.hpp"



int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  cv::startWindowThread();
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}
