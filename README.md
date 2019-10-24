# color-tracking

ROS node which converts a ROS image message topic into a cv::Mat, detects regions of a specific color (in this case red), and draws bounding boxes around them. 

## Installing dependencies
OpenCV >= 3.4.0: https://github.com/opencv/opencv

ROS - Kinetic Kame: http://wiki.ros.org/kinetic

```sh
sudo apt install ros-kinetic-camera-info-manager
sudo apt install ros-kinetic-cv-bridge
```

Usage: 

1. Clone this repo to your src-folder in your catkin workspace.

2. To run this node:

```bash
$ rosbag play -l <path-to-bagfile>
```

```bash
$ rosrun image_converting image_converting
```
If the image topic is of type compressed, run:
```bash
$ rosrun image_converting image_converting _image_transport:=compressed
```
