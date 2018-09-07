#!/usr/bin/env python

import rospy
import tf

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose


class ModelController(object):
	def __init__(self):
		self.set_state_srv_ = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

	def goto(self, model, x, y, yaw):
		quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

		pose = Pose()
		pose.position.x = x
		pose.position.y = y
		pose.position.z = 0
		pose.orientation.x = quaternion[0]
		pose.orientation.y = quaternion[1]
		pose.orientation.z = quaternion[2]
		pose.orientation.w = quaternion[3]

		state = ModelState()
		state.model_name = model
		state.pose = pose

		try:
		    ret = self.set_state_srv_(state)
		    print("[ModelController]: {}".format(ret.status_message))
		except Exception, e:
		    rospy.logerr('Error on calling service: %s',str(e))