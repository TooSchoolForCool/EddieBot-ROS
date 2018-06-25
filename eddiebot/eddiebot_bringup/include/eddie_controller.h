/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Haikal Pribadi <haikal.pribadi@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the Haikal Pribadi nor the names of other
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _EDDIE_CONTROLLER_H
#define	_EDDIE_CONTROLLER_H

#include <ros/ros.h>
#include <semaphore.h>
#include <std_msgs/Bool.h>
#include <eddiebot_msgs/Accelerate.h>
#include <eddiebot_msgs/Distances.h>
#include <eddiebot_msgs/DriveWithDistance.h>
#include <eddiebot_msgs/DriveWithPower.h>
#include <eddiebot_msgs/DriveWithSpeed.h>
#include <eddiebot_msgs/GetHeading.h>
#include <eddiebot_msgs/GetStatus.h>
#include <eddiebot_msgs/ResetEncoder.h>
#include <eddiebot_msgs/Rotate.h>
#include <eddiebot_msgs/StopAtDistance.h>
#include <eddiebot_msgs/Velocity.h>
#include <eddiebot_msgs/Voltages.h>

class EddieController {
public:
    EddieController();
    void execute();

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber velocity_sub_;
    ros::Subscriber ping_distances_sub_;
    ros::Subscriber ir_distances_sub_;
    ros::ServiceServer eddie_status_srv_;
    ros::ServiceClient eddie_drive_power_;
    ros::ServiceClient eddie_drive_speed_;
    ros::ServiceClient eddie_acceleration_rate_;
    ros::ServiceClient eddie_turn_;
    ros::ServiceClient eddie_stop_;
    ros::ServiceClient eddie_heading_;
    ros::ServiceClient eddie_reset_;

    sem_t mutex_execute_;
    sem_t mutex_interrupt_;
    sem_t mutex_state_;
    sem_t mutex_ping_;
    sem_t mutex_ir_;
    
    int left_power_, right_power_, rotation_power_;
    int acceleration_power_, deceleration_power_, min_power_;
    int left_speed_, right_speed_, rotation_speed_, acceleration_speed_;
    double linear_scale_, angular_scale_;
    int8_t left_, right_;
    int8_t current_power_;
    int16_t current_angle_, angular_;
    bool rotate_;
    bool interrupt_;
    bool process_;
    bool ping_distances_okay_, ir_distances_okay_;
    ros::Time last_cmd_time_;

    void velocityCallback(const eddiebot_msgs::Velocity::ConstPtr& message);
    void distanceCallback(const eddiebot_msgs::Distances::ConstPtr& message);
    void irCallback(const eddiebot_msgs::Voltages::ConstPtr& message);
    bool getStatus(eddiebot_msgs::GetStatus::Request& req,
            eddiebot_msgs::GetStatus::Response& res);
    void stop();
    void setAccelerationRate(int rate);
    void moveLinear(float linear);
    void moveAngular(int16_t angular);
    void moveLinearAngular(float linear, int16_t angular);
    void drive(int8_t left, int8_t right);
    void rotate(int16_t angular);
    void updatePower(int8_t left, int8_t right);
    int8_t clipPower(int power_unit, float linear);
    int16_t clipSpeed(int speed_unit, float linear);
};

#endif	/* _EDDIE_CONTROLLER_H */

