#!/usr/bin/env python
import rospy
import actionlib
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion


class RobotController(object):
	def __init__(self):
		self.move_base_client_ = actionlib.SimpleActionClient('move_base', MoveBaseAction)


	def goto(self, x, y, yaw=None):
		self.move_base_client_.wait_for_server()

		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = x
		goal.target_pose.pose.position.y = y

		if yaw is not None:
			quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
			goal.target_pose.pose.orientation.x = quaternion[0]
			goal.target_pose.pose.orientation.y = quaternion[1]
			goal.target_pose.pose.orientation.z = quaternion[2]
			goal.target_pose.pose.orientation.w = quaternion[3]

		self.move_base_client_.send_goal(goal)

		wait = self.move_base_client_.wait_for_result()

		if not wait:
		    rospy.logerr("Action server not available!")
		    rospy.signal_shutdown("Action server not available!")
		else:
		    return self.move_base_client_.get_result()
