#include "eddie_odom.h"
#include <iostream>

using namespace std;


EddieOdomPublisher::EddieOdomPublisher()
{
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    encoders_sub_ = nh_.subscribe("/eddie/encoders_data", 1, 
        &EddieOdomPublisher::encoder_cb_, this);

    x_ = y_ = th_ = 0.0;
    
    prev_left_encoder_cnt_ = prev_right_encoder_cnt_ = 0;

    current_time_ = last_time_ = ros::Time::now();
}

void EddieOdomPublisher::encoder_cb_(const eddiebot_msgs::Encoders::ConstPtr &msg)
{
    current_time_ = ros::Time::now();
    double dt = (current_time_ - last_time_).toSec();

    int delta_left_cnt = msg->left - prev_left_encoder_cnt_;
    int delta_right_cnt = msg->right - prev_right_encoder_cnt_;

    double delta_th = 1.0 * (delta_right_cnt - delta_left_cnt) * DISTANCE_PER_COUNT / WHEEL_BASE;
    double delta_dist = 0.5 * (delta_right_cnt + delta_left_cnt) * DISTANCE_PER_COUNT;
    double delta_x = delta_dist * cos(delta_th);
    double delta_y = delta_dist * sin(delta_th);

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    if(th_ > PI)
        th_ -= PI * 2;
    else if(th_ <= -PI)
        th_ += PI * 2;

    prev_left_encoder_cnt_ = msg->left;
    prev_right_encoder_cnt_ = msg->right;
    last_time_ = current_time_;

    publish_odom_(delta_x, delta_y, delta_th, dt);
}

void EddieOdomPublisher::publish_odom_(double dx, double dy, double dth, double dt)
{
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    odom_broadcaster_.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time_;
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    //set the velocity
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = dx / dt;
    odom.twist.twist.linear.y = dy / dt;
    odom.twist.twist.angular.z = dth / dt; 
    
    //publish the message
    odom_pub_.publish(odom);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "eddie_odom");

    EddieOdomPublisher eddie_odom_pub;

    ros::spin();

    return 0;
}

