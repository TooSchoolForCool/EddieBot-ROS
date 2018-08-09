#ifndef _ROOM_SEGMENTATION_CLIENT_H
#define _ROOM_SEGMENTATION_CLIENT_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cv_bridge/cv_bridge.h>
#include <room_segmentation/MapSegmentationAction.h>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


typedef actionlib::SimpleActionClient<room_segmentation::MapSegmentationAction> Client;

class RoomSegmentationClient
{
public:
    RoomSegmentationClient();
    void launch();

private:
    cv::Mat map_parser_(const std::vector<signed char> &data, int width, int height);
    cv::Mat segment_room_(cv::Mat &img, double resolution, int x, int y);
    void save2json_(room_segmentation::MapSegmentationResultConstPtr result);

private:
    ros::NodeHandle nh_;
};

#endif