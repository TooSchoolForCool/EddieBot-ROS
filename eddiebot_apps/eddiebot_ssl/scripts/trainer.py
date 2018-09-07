#!/usr/bin/env python
import json
import random

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

from model_controller import ModelController
from robot_controller import RobotController


def set_person_pose(mc, person_loc, room_id):
    idx = random.randint(0, len(person_loc[str(room_id)]) - 1)
    loc = person_loc[str(room_id)][idx]

    x = loc["x"]
    y = loc["y"]
    yaw = 1.0 * random.randint(-3, 3)

    mc.goto("person_standing_0", x, y, yaw)

    return x, y, idx


def explore_room(rc, room_dst, room_id):
    trace = room_dst[str(room_id)]

    for next_stop in trace:
        x = next_stop['x']
        y = next_stop['y']
        yaw = next_stop["yaw"]

        rc.goto(x, y, yaw)


def start_training(trace_history, room_dst, person_loc, approach_person, sound_wave_model):
    rc = RobotController()
    mc = ModelController()
    
    for i, trace in enumerate(trace_history):
        print("-" * 60)
        print("Current Time: {}, Sample: {}".format(rospy.get_rostime().secs, i))

        target_room = trace["target"]
        # robot back to origin
        explore_room(rc, room_dst, trace["origin"])
        # move a person to dst and emit sound
        px, py, pidx = set_person_pose(mc, person_loc, target_room)
        mc.spawn_model("sound_wave", sound_wave_model, px, py, 2)
        rospy.sleep(3)
        mc.delete_model("sound_wave")

        # robot actively explore the room according to the ranking result
        for next_room in trace["trace"]:
            if next_room == target_room:
                print("Sample {} find target room: {}".format(i, next_room))
                app_pos = approach_person[str(target_room)][pidx]
                rc.goto(app_pos["x"], app_pos["y"], app_pos["yaw"])
                rospy.sleep(1)
            else:
                print("Sample {} explore room: {}".format(i, next_room))
                explore_room(rc, room_dst, next_room)


if __name__ == '__main__':
    rospy.init_node('eddiebot_ssl_trainer')

    trace_file = rospy.get_param("~trace_file")
    json_data=open(trace_file).read()
    trace_history = json.loads(json_data)

    room_dst = rospy.get_param("~room_dest")
    json_data=open(room_dst).read()
    room_dst = json.loads(json_data)

    person_loc = rospy.get_param("~person_loc")
    json_data=open(person_loc).read()
    person_loc = json.loads(json_data)

    approach_person = rospy.get_param("~approach_person")
    json_data=open(approach_person).read()
    approach_person = json.loads(json_data)

    sound_wave_sdf = rospy.get_param("~sound_wave_model")
    sound_wave_model = None
    with open(sound_wave_sdf, "r") as f:
        sound_wave_model = f.read()

    start_training(trace_history, room_dst, person_loc, approach_person, sound_wave_model)