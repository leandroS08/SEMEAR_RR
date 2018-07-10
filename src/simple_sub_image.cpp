#include "ros/ros.h"
#include "std_msgs/String.h"

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>

using namespace cv;
using namespace std;

image_transport::Publisher image_pub_;


void chatterCallback(const sensor_msgs::ImageConstPtr &msg)
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
    
    image = cv_ptr->image;
    

    if( image.empty() ){
        return ;
    }
    
    waitKey(100);
    //imshow("Source", image);

    GaussianBlur( image, src, Size(3, 3), 0, 0, BORDER_DEFAULT );

    cvtColor(src, src_gray, COLOR_RGB2GRAY);
    
    int ksize = -1;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    // Second, use Sobel
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    
    Sobel(src_gray, grad_x, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT);
    Sobel(src_gray, grad_y, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT);

    // converting back to CV_8U
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);

    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, cv_ptr->image);

    //imshow("Source", image);
    cv_ptr->encoding = "mono8";
    
    image_pub_.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    image_transport::ImageTransport it_(n);

    ros::Subscriber sub = n.subscribe("/AMR/camera", 1000, chatterCallback);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
    ros::spin();

    return 0;
}
