
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
static const std::string OPENCV_WINDOW6 = "Histogram_bgr";
static const std::string OPENCV_WINDOW7 = "Histogram_gray";
static const std::string OPENCV_WINDOW8 = "Convex hull";
static const std::string OPENCV_WINDOW9 = "K-means";

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
    //image_sub_ = it_.subscribe("/manta/manta/camerafront/camera_image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW1);
    cv::namedWindow(OPENCV_WINDOW2);
    cv::namedWindow(OPENCV_WINDOW3);
    cv::namedWindow(OPENCV_WINDOW4);
    cv::namedWindow(OPENCV_WINDOW5);
    cv::namedWindow(OPENCV_WINDOW6);
    cv::namedWindow(OPENCV_WINDOW7);
    cv::namedWindow(OPENCV_WINDOW8);
    cv::namedWindow(OPENCV_WINDOW9);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
    cv::destroyWindow(OPENCV_WINDOW3);
    cv::destroyWindow(OPENCV_WINDOW4);
    cv::destroyWindow(OPENCV_WINDOW5);
    cv::destroyWindow(OPENCV_WINDOW6);
    cv::destroyWindow(OPENCV_WINDOW7);
    cv::destroyWindow(OPENCV_WINDOW8);
    cv::destroyWindow(OPENCV_WINDOW9);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   //std::cout << "callback " << std::endl; 
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

    /*cv::Mat gray;
    cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);

    
*/
    // K-means

    /* cv::Mat reshaped_img = image.reshape(1, image.cols * image.rows);
        cv::Mat reshaped_img32f;
        reshaped_img.convertTo(reshaped_img32f, CV_32FC1, 1.0 / 255.0); */

    int K = 4;
    cv::Mat labels;
    cv::Mat centers;
    cv::Mat img = cv_ptr->image;
    cv::Mat reshaped_img = img.reshape(1, img.cols * img.rows);
    cv::Mat img32f;
    reshaped_img.convertTo(img32f, CV_32FC1, 1.0 / 255.0);

    double segmentation = kmeans(img32f, K, labels,  TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);

    cv::Mat rgb_image(img.rows, img.cols, CV_8UC3);
    cv::MatIterator_<cv::Vec3b> rgb_first = rgb_image.begin<cv::Vec3b>();
    cv::MatIterator_<cv::Vec3b> rgb_last = rgb_image.end<cv::Vec3b>();
    cv::MatConstIterator_<int> label_first = labels.begin<int>();
 
    cv::Mat centers_u8;
    centers.convertTo(centers_u8, CV_8UC1, 255.0);
    cv::Mat centers_u8c3 = centers_u8.reshape(3);
 
    while ( rgb_first != rgb_last ) {
      const cv::Vec3b& rgb = centers_u8c3.ptr<cv::Vec3b>(*label_first)[0];
      *rgb_first = rgb;
      ++rgb_first;
      ++label_first;
     }

    // HSV filtering

    cv::Mat red1, red2, blurry, detected_edges;
    Rect2d bbox;
    inRange(hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), red1);
    inRange(hsv, Scalar(160,100,100), Scalar(179,255,255), red2);
    red1 = red1 + red2;

    // Finding contours
    GaussianBlur(red1, blurry, Size(13,13),0,0);
    Canny(blurry, detected_edges, 10, 50, 3);
    vector<vector<Point> > contours;
    findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Convex hull creation

    vector< vector<Point> > hull(contours.size());
    for(int i = 0; i < contours.size(); i++){
      convexHull(Mat(contours[i]), hull[i], false);
    }

    // Draw contours

    Mat drawing = Mat::zeros(detected_edges.size(), CV_8UC3); 

    for(int i = 0; i < contours.size(); i++){
      // Scalar color_contours = Scalar(0, 255, 0); // green contours
      Scalar color = Scalar(255, 0, 0); // Blue convex hull
      // Draw ith contour
      // Scalar color = Scalar(255, 0, 0);

      // Draw ith hull
      drawContours(drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point());
    }

    // Initializing the histogram parameters
    int histSize = 256;
    cv::Mat gray, gray_hist;

    // convert to grayscale
    cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);


    float range[] = { 0, 256 };
    const float* histRange = { range };
    bool uniform = true; bool accumulate = false;
    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/histSize );
    cv::Mat histImage_g( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

    // Computing the histogram - grayscale
    calcHist(&gray, 1, 0, cv::Mat(), gray_hist,
  	   1, &histSize, &histRange, uniform, accumulate );

           // Normalize the result to [ 0, histImage.rows ]
    normalize(gray_hist, gray_hist, 0, histImage_g.rows, NORM_MINMAX, -1, cv::Mat());

   

    // Accummulation
    for (int i = 0; i < histSize; i++) {
  		line( histImage_g, Point( bin_w*(i-1), hist_h - cvRound(gray_hist.at<float>(i-1)) ),
  		Point( bin_w*(i), hist_h - cvRound(gray_hist.at<float>(i)) ),
  		Scalar(255, 255, 255), 2, 8, 0  );
    }


    // Computing the histogram - BGR
    cv::Mat b_hist, g_hist, r_hist;
    vector<Mat> bgr_planes;
    split(cv_ptr->image, bgr_planes );
    calcHist( &bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist( &bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );



    /// Normalize the result to [ 0, histImage.rows ]
    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
    normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    /// Draw for each channel
    for( int i = 1; i < histSize; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                        Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                        Scalar( 255, 0, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                        Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                        Scalar( 0, 255, 0), 2, 8, 0  );
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                        Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                        Scalar( 0, 0, 255), 2, 8, 0  );
    }





    for (int j = 0; j < contours.size(); j++){

      cv::rectangle(cv_ptr->image, boundingRect(contours[j]), CV_RGB(0,0,255), 10);
      //bbox = boundingRect(contours[i]);
    }

    //cv::rectangle(cv_ptr->image, std::vector bbox, CV_RGB(0,0,255));

    // Update GUI Window
   
    cv::imshow(OPENCV_WINDOW1, cv_ptr->image); //cv_ptr->image
    cv::imshow(OPENCV_WINDOW2, red1);
    cv::imshow(OPENCV_WINDOW3, detected_edges);
    cv::imshow(OPENCV_WINDOW4, blurry); 
    cv::imshow(OPENCV_WINDOW5, gray);
    cv::imshow(OPENCV_WINDOW6, histImage);
    cv::imshow(OPENCV_WINDOW7, histImage_g);
    cv::imshow(OPENCV_WINDOW8, drawing);
    cv::imshow(OPENCV_WINDOW9, rgb_image);

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
