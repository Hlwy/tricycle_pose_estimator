#!/usr/bin/python
import argparse
import numpy as np
import os
import matplotlib.pyplot as plt
from classes import *
from utils import utils as ut

if __name__ == "__main__" :

	print ("\n" * 10000)
	print("=============================================")
	np.set_printoptions(suppress=True)
	np.set_printoptions(precision=4)
	# Setup commandline argument(s) structures
	ap = argparse.ArgumentParser(description='Image Segmentation')

	# ap.add_argument("--inputs", "-i", type=str, default='../data/processed/trike_straight_extracted_inputs.csv', metavar='FILE', help="Name of video file to parse")
	# ap.add_argument("--truth", "-t", type=str, default='../data/processed/trike_straight_extracted_truth.csv', metavar='FILE', help="Name of video file to parse")
	# ap.add_argument("--output", "-o", type=str, default='../data/processed/trike_straight_generated_outputs.csv', metavar='FILE', help="Name of video file to parse")

	ap.add_argument("--inputs", "-i", type=str, default='../data/processed/trike_circle_extracted_inputs.csv', metavar='FILE', help="Name of video file to parse")
	ap.add_argument("--truth", "-t", type=str, default='../data/processed/trike_circle_extracted_truth.csv', metavar='FILE', help="Name of video file to parse")
	ap.add_argument("--output", "-o", type=str, default='../data/processed/trike_circle_generated_outputs.csv', metavar='FILE', help="Name of video file to parse")

	# Store parsed arguments into array of variables
	args = vars(ap.parse_args())
	# Extract stored arguments array into individual variables for later usage in script
	_inputsPath = args["inputs"]
	_truthPath = args["truth"]
	_outPath = args["output"]
	# Initialize storage containers
	_outputs = []

	# Initialize useful class objects
	model = TricycleModel()
	filter = EKF(7,4)

	# Tune EKF Parameters before attaching EKF to model
	testR = np.array([[1.0, 0.7, 1.0, 0.9],]).T
	testQ = np.array([[0.5, 0.5, 0.01, 1.0, 0.01, 1.0, 1.0],]).T
	filter.setR(np.diagonal(testR))
	filter.setQ(np.diagonal(testQ))
	# Attach EKF to Mobile-Base
	model.attachEstimationMethod(filter) # Link the model with the estimation method

	# Read .csv file for estimation inputs and ground truths in as array of values for usage
	inputs = np.genfromtxt(_inputsPath, delimiter=',')
	truths = np.genfromtxt(_truthPath, delimiter=',')

	# Extract import bits of data for usage in setting initial conditions
	t0 = inputs[0,0]; steer0 = inputs[0,1]; rev0 = inputs[0,2]; gyro0 = inputs[0,3]
	# Determine limits needed for looping
	nEntries,nInputs = inputs.shape
	print("")

	# Go through each data entry and calculate our estimated pose
	# for i in range(0,nEntries):
	for i in range(115,300):
		# Grab current data entries
		time = inputs[i,0];
		steering_angle = inputs[i-1,1]
		encoder_angle = inputs[i-1,2]
		gyroz = inputs[i-1,3]

		pose = model.estimate(time,steering_angle,encoder_angle,gyroz)
		truth = truths[i,:]
		outRow = [pose[0], pose[1], pose[2], truth[0], truth[1], truth[2] ]
		_outputs.append(outRow)

	outputs = np.array(_outputs)
	# Save results to file for post-analysis
	np.savetxt(_outPath,outputs, delimiter=',',header='predX, predY, predHeading, trueX, trueY, trueHeading')
