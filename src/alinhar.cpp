#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"

ros::Publisher state_pub;
ros::Publisher pubVelocidade;

const double VEL = 0.08;

// Encaminhando o erro de deslocamento como "estado"  para o PID
void callbackShift(const std_msgs::Float32 &message)
{
    std_msgs::Float64 shift;
    shift.data = message.data;
    state_pub.publish(shift);
}

void control_effort_callback(const std_msgs::Float64ConstPtr &msg)
{
    geometry_msgs::Twist twist_msg;

    twist_msg.linear.x = VEL;
    twist_msg.linear.y = 0;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = msg->data;

    pubVelocidade.publish(twist_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Alinhando");

    ros::NodeHandle nh;
    ros::Subscriber subShift = nh.subscribe("/shift_central_topic", 1000, callbackShift);

    state_pub = nh.advertise<std_msgs::Float64>("/state", 5);
    ros::Publisher setpoint_pub = nh.advertise<std_msgs::Float64>("/setpoint", 1);
    ros::Subscriber control_effor_sub = nh.subscribe("/control_effort", 1, control_effort_callback);

    pubVelocidade = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // Publicando o SetPoint ( Erro 0 )
    std_msgs::Float64 setpoint_msg;
    setpoint_msg.data = 0;
    ros::Duration(1).sleep();
    setpoint_pub.publish(setpoint_msg);

    ros::spin();
    ROS_INFO_STREAM(subShift);

    return 0;
}
