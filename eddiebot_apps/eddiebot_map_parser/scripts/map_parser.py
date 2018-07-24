#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid

import cv2
import json
import numpy as np

def callback(msg):
    """callback function for /map topic

    Param:
        msg: ros datatype nav_msgs/OccupancyGrid.msg
            msg.header  --> header of the map msg
            msg.info    --> map meta-information
                msg.resolution  --> resolution of the map (meter / pixel)
                msg.info.width  --> map width
                msg.info.height --> map height
                msg.info.origin --> map origin
                    msg.info.origin.position    --> real world pose of the cell(0, 0)
            msg.data    --> Map information
                            Occupancy probabilities are in the range [0,100].
                            Unknown is -1.
    """
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    pos = msg.info.origin.position

    data = msg.data

    # map_access(0, 0) is the left-bottom corner
    # i is the row index
    # j is the column index
    map_access = lambda i, j : data[i * width + j]

    map_data = [[0 for j in range(0, width)] for i in range(0, height)]

    for i in range(0, height):
        for j in range(0, width):
            if(map_access(i, j) == 100):
                # convert (0, 0) to upper-left corner
                map_data[height - i - 1][j] = 100
            elif(map_access(i, j) == 0):
                map_data[height - i - 1][j] = 0
            else:
                map_data[height - i - 1][j] = -1


    json_data = {}
    json_data["width"] = width
    json_data["height"] = height
    json_data["resolution"] = resolution
    json_data["origin"] = (pos.x, pos.y)
    json_data["data"] = map_data

    with open('map_data.json', 'w') as outfile:
        json.dump(json_data, outfile)

    rospy.signal_shutdown("Map parsing is done.")


def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # disable_signals indicate this node could be shutdown manually
    rospy.init_node('map_parser', anonymous=True, disable_signals=True)
    rospy.Subscriber('/map', OccupancyGrid, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()