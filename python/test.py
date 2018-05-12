import argparse
import numpy as np
from utils import utils as ut

def estimate(time,steering_angle,encoder_ticks,angular_velocity):
	"""
    :param time:
		Time​ (seconds) of ​reading ​of ​the ​input​ data
    :param steering_angle:
		The angle (radians) of the steering wheel
    :param encoder_ticks:
		The number of ticks (integer) from the traction (drive) motor encoder
    :param angular_velocity:
		The angular velocity (radians / second) read from a gyroscope about the Z-axis
    :return:
		The estimated pose [x, y, heading] of the specific model in units of (meters, meters, radians) respectively
    """
	pose_hat = [0, 0, 0]



	return pose_hat


if __name__ == "__main__" :

	# Setup commandline argument(s) structures
	ap = argparse.ArgumentParser(description='Image Segmentation')
	ap.add_argument("--data", "-d", type=str, default='combined_test_data.csv', metavar='FILE', help="Name of video file to parse")
	# Store parsed arguments into array of variables
	args = vars(ap.parse_args())

	# Extract stored arguments array into individual variables for later usage in script
	_data = args["data"]

	inputs = np.genfromtxt(_data, delimiter=',')
	imuTs = inputs[:,0]
	imuWzs = inputs[:,1]
	encTs = inputs[:,2]
	encSteers = inputs[:,3]
	encRevs = inputs[:,4]

	nEntries, nInputs, _ = inputs.shape
	print inputs.shape

	# For loop going through each time step
	for i in range(nEntries):
		imuT = inputs[i,0]
		imuWz = inputs[i,1]
		encT = inputs[i,2]
		encSteer = inputs[i,3]
		encRev = inputs[i,4]



	print inputs[:,0]
