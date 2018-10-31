#include <geometry_msgs/Twist.h>
#include<math.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h"

std_msgs::Float32 shift;

void callbackShift(const std_msgs::Float32 &message){
    shift = message; 
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Alinhando");

    ros::NodeHandle nh;
    ros::Subscriber subShift = nh.subscribe("/shift_central_topic", 1000, callbackShift);
    ros::Publisher pubVelocidade = nh.advertise<geometry_msgs::Twist>("McQueen/cmd_vel", 1000);

    ROS_INFO("Alinhando");

    ros::spin();
    ROS_INFO_STREAM(subShift);
    
    return 0;
}

