#include "eddie_vel_controller.h"

using namespace std;

EddieVelController::EddieVelController()
{
    vel_pub_ = nh_.advertise<eddiebot_msgs::Velocity>("/eddie/cmd_vel", 5);

    cmd_vel_sub_ = nh_.subscribe("raw_cmd_vel", 1, 
        &EddieVelController::cmd_vel_callback_, this);
}

void EddieVelController::cmd_vel_callback_(const geometry_msgs::Twist::ConstPtr &msg)
{
    eddiebot_msgs::Velocity cmd_vel;

    cmd_vel.linear = msg->linear.x;
    cmd_vel.angular = msg->angular.z;

    vel_pub_.publish(cmd_vel);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "eddie_vel_controll");

    EddieVelController eddie_vel_controller;

    ros::spin();

    return 0;
}

