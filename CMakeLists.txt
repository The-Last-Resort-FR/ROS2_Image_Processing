cmake_minimum_required(VERSION 3.8)
project(image_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(sources
  "src/image_processor.cpp"
  "src/ImageProcessor.cpp")

set(includes
  "include/ImageProcessor.hpp")

  
  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(cv_bridge REQUIRED)
  
  
  add_executable(image_processor ${sources} ${includes})
  include_directories(AFTER SYSTEM "/usr/include")

ament_target_dependencies(image_processor
  rclcpp
  sensor_msgs
  image_transport
  OpenCV
  cv_bridge
)

target_link_libraries(image_processor ${OpenCV_LIBS})

target_include_directories(image_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(image_processor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS image_processor
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
