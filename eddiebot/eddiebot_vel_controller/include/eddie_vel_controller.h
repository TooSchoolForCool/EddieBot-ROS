#ifndef _EDDIE_VEL_CONTROLLER_H
#define _EDDIE_VEL_CONTROLLER_H

#include <ros/ros.h>
#include <eddiebot_msgs/Velocity.h>
#include <geometry_msgs/Twist.h>


class EddieVelController
{
public:
    EddieVelController();
    
private:
	void cmd_vel_callback_(const geometry_msgs::Twist::ConstPtr &msg);

private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber cmd_vel_sub_;
};

#endif