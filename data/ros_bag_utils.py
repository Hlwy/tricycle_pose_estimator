#!/usr/bin/env python

"""

!!!!TODO!!!!

---- Comment this script for line-by-line logic

---- POSSIBLY make more adaptable to other people's uses and topics

"""

import rosbag
import sys
import os
import roslib
import rosbag
import rospy
import numpy as np
import csv

from tf.transformations import quaternion_from_euler, rotation_matrix, quaternion_from_matrix, euler_from_quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion

def convert_pose_to_xy_and_theta(pose):
	""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
	orientation_tuple = (	pose.orientation.x,
							pose.orientation.y,
							pose.orientation.z,
							pose.orientation.w)
	angles = euler_from_quaternion(orientation_tuple)
	return (pose.position.x, pose.position.y, angles[2])


def extract_bag_topics(bagfile, _topics):
	bag = rosbag.Bag(bagfile)
	fusionData = []; truthData = []
	truePose = [0,0,0];	gyro_z = 0; gyro_sum = 0; steering_angle = 0; enc_rotations = 0
	imuCount = 0; jointCount = 0; truthCount = 0
	time = 0

	for topic, msg, t in bag.read_messages(topics=_topics):
		if topic == "/Brain_Tricycle/joint_states":
			jointCount += 1
		if topic == "/tricycle/imu":
			imuCount += 1
		if topic == "/gazebo/model_states":
			truthCount += 1

	topicCount = [jointCount,imuCount,truthCount]
	imuCount = 0; jointCount = 0; truthCount = 0
	print str(topicCount)

	for topic, msg, t in bag.read_messages(topics=_topics):
		if topic == "/Brain_Tricycle/joint_states":
			time = float(str(msg.header.stamp))/1000000000 # Probably for efficient way to do this
			steering_angle = msg.position[0]
			enc_rotations = msg.position[1]
			if imuCount == 0:
				imuCount = 1
			avg_gyro = gyro_sum / float(imuCount)
			jointCount += 1
			imuCount = 0
			gyro_sum = 0
			tmpData = [time, steering_angle, enc_rotations, avg_gyro]
			# print("Time: " + str(time))
			# print [str(gyro_z), str(avg_gyro)]
			fusionData.append(tmpData)
			truthData.append(truePose)
		if topic == "/tricycle/imu":
			gyro_z = msg.angular_velocity.z
			gyro_sum = gyro_sum + gyro_z
			imuCount += 1
		if topic == "/gazebo/model_states":
			truePose = convert_pose_to_xy_and_theta(msg.pose[1])
			# print(str(truePose))

	bag.close()

	fusionData = np.array(fusionData, dtype=np.float)
	truthData = np.array(truthData, dtype=np.float)

	print fusionData.shape
	print truthData.shape

	return fusionData, truthData

if __name__ == '__main__':
	# if len(sys.argv) <= 2:
	# 	print 'python extract_steering_angles.py <bag file> <topic>'
	# sys.exit()
	# bagfile = sys.argv[1]
	# fileIn = "trike_straight"
	fileIn = "trike_circle"
	bagfile = str(fileIn) + ".bag"
	topics = ["/tricycle_base/joint_states", "/gazebo/model_states", "/tricycle/imu"]
	inputlog = str(fileIn) + "_extracted_inputs.csv"
	truthlog = str(fileIn) + "_extracted_truth.csv"
	inputs, truth = extract_bag_topics(bagfile, topics)

	np.savetxt(inputlog, inputs, fmt='%.4f', delimiter=',', header="Time (sec), Steering Angle (rad), Front Wheel Angle (rad), Gyroscope Z (rad / sec)")
	np.savetxt(truthlog, truth, fmt='%.4f', delimiter=',', header="TrueX (m), TrueY (m), TrueHeading (rad)")
