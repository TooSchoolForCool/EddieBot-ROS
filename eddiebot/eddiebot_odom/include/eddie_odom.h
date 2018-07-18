#ifndef _EDDIE_ODOM_H
#define _EDDIE_ODOM_H

#include <ros/ros.h>
#include <eddiebot_msgs/Encoders.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define PI      3.14159265359
#define TWOPI   (PI * 2)
// encoder counter per revolution
#define COUNTS_PER_REVOLUTION   144
// Wheel Radius
#define WHEEL_RADIUS    0.1524
// the distance of a wheel move forward when encoder increased by 1
#define DISTANCE_PER_COUNT      ((PI * WHEEL_RADIUS) / COUNTS_PER_REVOLUTION)
// two wheels center-to-center distance
#define WHEEL_BASE      0.39


class EddieOdomPublisher
{
public:
    EddieOdomPublisher();
    
private:
    void encoder_cb_(const eddiebot_msgs::Encoders::ConstPtr &msg);
    void publish_odom_(double dx, double dy, double dth, double dt);

private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber encoders_sub_;
    tf::TransformBroadcaster odom_broadcaster_;

    // previous position
    double x_;
    double y_;
    double th_;

    // previous encoders count
    int prev_left_encoder_cnt_;
    int prev_right_encoder_cnt_;

    ros::Time current_time_;
    ros::Time last_time_;
};

#endif