
// Dependencies
#include <opencv2/opencv.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc_c.h"
#include <sstream>
using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW1 = "Image window1";
static const std::string OPENCV_WINDOW2 = "Image window2";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/auv/bot_cam/image_color", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW1);
    cv::namedWindow(OPENCV_WINDOW2);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Converting BGR to HSV
    cv::Mat hsv;
    cvtColor(cv_ptr->image, hsv, COLOR_BGR2HSV);

    cv::Mat red1, red2, blurry, detected_edges;
    Rect2d bbox;
    inRange(hsv, Scalar(0, 120, 70), Scalar(20, 255, 255), red1);
    inRange(hsv, Scalar(160,120,120), Scalar(180,255,255), red2);
    red1 = red1 + red2;

    // Finding contours
    GaussianBlur(red1, blurry, Size(5,5),0,0);
    Canny(blurry, detected_edges, 10, 50, 3);
    vector<vector<Point> > contours;
    findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++){
      cv::rectangle(cv_ptr->image, boundingRect(contours[i]), CV_RGB(0,0,255), 10);
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW1, cv_ptr->image); 
    cv::imshow(OPENCV_WINDOW2, red1);

    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
