/**
 * @file image_processor.cpp
 * @author tlr
 * @brief Entry point
 * @version 0.0.0
 * @date 2025-04-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <cstdio>
#include "ImageProcessor.hpp"


/**
 * @brief Spin a node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  cv::startWindowThread();
  rclcpp::spin(std::make_shared<ImageProcessor>());
  rclcpp::shutdown();
  return 0;
}
