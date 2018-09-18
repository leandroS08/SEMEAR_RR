#include "ros/ros.h"
#include "std_msgs/String.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

image_transport::Publisher image_pub_;

int thresh = 200;
// Oi
int max_thresh = 255;
const char *source_window = "Source image";
const char *corners_window = "Corners detected";

void wrapper_variable(int, void *){}


void cornerHarris_demo(const sensor_msgs::ImageConstPtr &msg)
{
    Mat image, src, src_gray;
    
    Mat grad;
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
    
    src = cv_ptr->image;
    

    if( src.empty() ){
        return ;
    }
    cvtColor(src, src_gray, COLOR_BGR2GRAY);

    Mat dst, dst_norm, dst_norm_scaled;
    dst = Mat::zeros(src.size(), CV_32FC1);
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    
    cornerHarris(src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
    normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
    convertScaleAbs(dst_norm, dst_norm_scaled);
    for (int j = 0; j < dst_norm.rows; j++)
    {
        for (int i = 0; i < dst_norm.cols; i++)
        {
            if ((int)dst_norm.at<float>(j, i) > thresh)
            {
                circle(src_gray, Point(i, j), 5, Scalar(0), 2, 8, 0);
            }
        }
    }

    cv_ptr->image = src_gray;
    cv_ptr->encoding = "mono8";
    image_pub_.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    image_transport::ImageTransport it_(n);

    ros::Subscriber sub = n.subscribe("/AMR/camera", 1000, cornerHarris_demo);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    namedWindow( source_window, WINDOW_AUTOSIZE );
    createTrackbar("Threshold: ", source_window, &thresh, max_thresh, wrapper_variable);
    imshow( source_window, imread("/home/gonzales/Pictures/house.jpeg", IMREAD_COLOR )); 
    ros::Rate rate(10) ;
    
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        waitKey(100);    
    }
    
    
    return 0;
}
