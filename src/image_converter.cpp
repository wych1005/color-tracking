
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

static const std::string OPENCV_WINDOW1 = "End result";
static const std::string OPENCV_WINDOW2 = "Mask";
static const std::string OPENCV_WINDOW3 = "Canny edge detector";
static const std::string OPENCV_WINDOW4 = "Gaussian blur";
static const std::string OPENCV_WINDOW5 = "Grayscale";
static const std::string OPENCV_WINDOW6 = "Histogram";

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
/*
    cv::namedWindow(OPENCV_WINDOW1);
    cv::namedWindow(OPENCV_WINDOW2);
    cv::namedWindow(OPENCV_WINDOW3);
    cv::namedWindow(OPENCV_WINDOW4);*/
    cv::namedWindow(OPENCV_WINDOW5);
    cv::namedWindow(OPENCV_WINDOW6);
  }

  ~ImageConverter()
  {/*
    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
    cv::destroyWindow(OPENCV_WINDOW3);
    cv::destroyWindow(OPENCV_WINDOW4);*/
    cv::destroyWindow(OPENCV_WINDOW5);
    cv::destroyWindow(OPENCV_WINDOW6);
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

    /* Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
*/

    // Converting BGR to HSV
    cv::Mat hsv;
    cvtColor(cv_ptr->image, hsv, COLOR_BGR2HSV);

    //cv::Mat gray;
    //cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);

    cv::Mat red1, red2, blurry, detected_edges;
    Rect2d bbox;
    inRange(hsv, Scalar(0, 70, 70), Scalar(40, 255, 255), red1);
    inRange(hsv, Scalar(150,70,120), Scalar(190,255,255), red2);
    red1 = red1 + red2;

    // Finding contours
    GaussianBlur(red1, blurry, Size(13,13),0,0);
    Canny(blurry, detected_edges, 10, 50, 3);
    vector<vector<Point> > contours;
    findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Initializing the histogram parameters
    int histSize = 256;
    cv::Mat gray, g_hist;
    cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

    float range[] = { 0, 256 };
    const float* histRange = { range };
    bool uniform = true; bool accumulate = false;
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    cv::Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

    // Computing the histogram
    calcHist(&gray, 1, 0, cv::Mat(), g_hist,
  	   1, &histSize, &histRange, uniform, accumulate );

    // Normalize the result to [ 0, histImage.rows ]
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, cv::Mat());

    // Accummulation
    for (int i = 0; i < histSize; i++) {
  		line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ),
  		Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
  		Scalar( 0, 255, 0), 2, 8, 0  );
    }

    for (int j = 0; j < contours.size(); j++){
      cv::rectangle(cv_ptr->image, boundingRect(contours[j]), CV_RGB(0,0,255), 10);
      //bbox = boundingRect(contours[i]);
    }

    //cv::rectangle(cv_ptr->image, std::vector bbox, CV_RGB(0,0,255));

    // Update GUI Window
    /*
    cv::imshow(OPENCV_WINDOW1, cv_ptr->image); //cv_ptr->image
    cv::imshow(OPENCV_WINDOW2, red1);
    cv::imshow(OPENCV_WINDOW3, detected_edges);
    cv::imshow(OPENCV_WINDOW4, blurry); */
    cv::imshow(OPENCV_WINDOW5, gray);
    cv::imshow(OPENCV_WINDOW6, histImage );

    //ShowManyImages(OPENCV_WINDOW, 2, cv_ptr->image, red1); //cv_ptr->image
    //cv::imshow(OPENCV_WINDOW, red1););
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
