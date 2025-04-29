# ROS2 Image Processing Node

A node that applies some processing to images

## Description

This program takes images from the camera_manager

## Getting Started

### Dependencies

- ROS2 Humble
- OpenCV
- CUDA

### Installing

Clone the repository  
```bash
git clone https://github.com/The-Last-Resort-FR/ROS2_Image_Processing.git
cd ROS2_Image_Processing
```
Build  
```bash
colcon build --symlink-install
```
Source the environement  
```bash
source install/setup.bash
```
The trigger source should be connected to line0 on the cameras (green wire on official M8 cable)

### Executing program

Run the node  
```bash
ros2 run image_processor image_processor 
```
Make sure the trigger source is actually triggering the cameras  
You can visualize the images on the topics depending on the configuration of the camera master
```bash
TODO

```

## TODO

- Launch file with param file
- Other image treatment for stereo vision
- Optimisation
- Documentation


## Version History

- V0.0.0 : Draft version

## License

This project is licensed under the "The Unlicense" License - see the LICENSE.md file for details

## Acknowledgments

- 