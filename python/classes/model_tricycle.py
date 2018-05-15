#!/usr/bin/env python
# coding=utf-8
import numpy as np

class TricycleModel(object):
	""" A Tricycle-model of a mobile platform, used to estimate the 2D pose
	(X, Y, heading) from the available sensors (namely encoders and gyro). """

	def __init__(self):
		""" Sets up all internally used variables and loads initial parameters """
		# Name used for external identification
		self.name = "Mobile-Base Model [Tricycle]"
		print("Initializing " + str(self.name))

		"""
		:brief Sensor Fusion Method
			Method used to estimate the tricycle model's current pose, based on
			available data.
		:default
			None - Dead-Reckoning
		"""
		self.estimation_method = None
		self.filter = None
		self.F = None
		self.H = np.array([	[0, 0, 0, 0, 0, 0, 1],
                          	[0, 0, 0, 0, 0, 1, 0],
                          	[0, 0, 0, 0, 1, 0, 0]
						  ])

		# Initial Variables
		self.prev_pose = [0, 0, 0]
		self.prev_states = [0, 0, 0, 0, 0, 0, 0]
		self.prev_time = 0
		self.prev_ticks = 0
		self.prev_steering_angle = 0

		# Initialize Counters
		self.counter = 0

		# Physical Parameters and Constraints
		#    (TODO: add functions for easier externally user-defined constraints)
		self.steering_angle_limit = np.pi / 2.0 # Defines +/- steering angle limit in radians
		self.r_dist = 1.0 		# Distance b/w front and rear axis (meters)
		self.d_dist = 0.75		# Distance b/w rear wheels (meters)
		self.rear_radius = 0.2  # (meters)
		self.front_radius = 0.2 # (meters)
		self.ticks_per_rev = 512 # unit-less

		# Derive conversion factor to convert ticks to meters (Should check for encoder configuration, in case of different model setup)
		self.ticks_per_meter = (2 * np.pi * self.front_radius) / self.ticks_per_rev

		# Steering threshold, or deadband, (threshold to ensure steering angle is zero for very small angles not achievable by controllers)
		self.steer_thresh = 0.02

		# TODO: Noise parameters
		self.mu = 0				# Wheel Friction Coefficient
		self.gyro_noise = 0		# Gyro Noise, and bias
		self.gyro_offset = None	# Innate gyro offset

		# Flags
		self.expect_processed_inputs = False	# Should we expect sensor inputs to be refined, or crude
		self.expect_encoder_revs = False		# Should we expect raw encoder data to be in revolutions (or radians)
		self.encoder_on_front = True			# Is the encoder used for wheel rotation measurements on the front wheel

	def setH(self, h):
		self.H = h

	def setEstimationMethod(self, method_name):
		if method_name == "EKF" or method_name == "ekf" or method_name == 1:
			self.estimation_method = 1
		elif method_name == "alternative method":
			self.estimation_method = 2
		else:
			self.estimation_method = None
			print("ERROR: Pose estimation method specified is not currently supported. Default Method being used: Dead-Reckoning")

	def attachEstimationMethod(self, filter):
		print("Attaching " + str(filter.name) + " to " + str(self.name))
		self.filter = filter

	def pre_process_inputs(self, raw_inputs, verbose=False):
		"""
		:brief
			The purpose of this function is to, essentially, saturate the raw inputs,
			or ensure that they aren't outside our user-defined physical constraints.
			NOTE: Expected to be called outside class.
		:param raw_inputs:
			Compaction of all the raw inputs for pose estimation in an array whose
			format is [time(secs), steering_angle(radians), encoder_revs(wheel rotations), angular_velocity(rads/sec)]
		:return:
			Saturated inputs of same format as raw_inputs, and [secs, radians, ticks, rads/sec] units respectively
		"""
		# Initialize internally used variables
		_conv_fact = 0		# Encoder rotations -> Ticks Conversion factor gain
		g_offset = 0
		processed_inputs = []

		# De-mux inputs for individual variable checking
		steer_ang, enc_revs, gyro_z = raw_inputs

		# Check if steering angle is outside our working limits regardless of input (Safety)
		if steer_ang < -self.steering_angle_limit:
			steer_ang = -self.steering_angle_limit
		elif steer_ang > self.steering_angle_limit:
			steer_ang = self.steering_angle_limit

		# Check if we should be expecting the rest of the inputs to be preprocessed
		if self.expect_processed_inputs == True:
			processed_inputs = [steer_ang, enc_revs, gyro_z]
		else:
			# Check if encoder inputs are expected to be revolutions, or radians
			if self.expect_encoder_revs == True:
				_conv_fact = 1
			else:
				_conv_fact = 1 / (2 * np.pi)
			# Convert raw wheel rotations into ticks
			enc_revs = self.ticks_per_rev * enc_revs * _conv_fact

			# Check if we need to process gyroscope data
			if self.gyro_offset == None:
				g_offset = 0
			else:
				g_offset = self.gyro_offset
			# Process gyro data
			gyro_z = gyro_z - g_offset

			# Package processed inputs
			processed_inputs = [steer_ang, enc_revs, gyro_z]

		# Debugging feature
		if verbose == True:
			print("Input Pre-Processing:")
			print("	Inputs Before Processing: " + str(raw_inputs))
			print("	Inputs After Processing: " + str(processed_inputs))

		return processed_inputs

	def estimate(self,time,steering_angle,encoder_ticks,angular_velocity, verbose=True):
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
		if verbose == True:
			print("--------- Estimate Update #" + str(self.counter) + " ---------")

		# Initialize internally used variables
		estimates = []
		# EKF-expected observation format: obs = [velocity_enc, heading_enc, heading_gyro]

		# Pre-process data if needed
		raw_inputs = [steering_angle,encoder_ticks,angular_velocity]
		fixed_inputs = self.pre_process_inputs(raw_inputs)

		# Handle any irregularities that may occur with time calculations on first step
		if self.counter == 0:
			self.prev_time = time
			self.prev_ticks = fixed_inputs[1]
			self.counter += 1
			dt = 0
			return self.prev_pose
			# print("----	----	Skipping first data entry with Time = " + str(time))
		# else:

		# Calculate time difference (sec) since last step
		dt = time - self.prev_time
		# Calculate the incremental encoder difference (ticks) since last step
		denc = fixed_inputs[1] - self.prev_ticks
		# Calculate the linear displacement (meters) since last step, based on change in encoder rotation
		d_distance = denc * self.ticks_per_meter
		# Convert encoder angular displacement into velocity for compatibility with state-transition models
		encoder_velocity = d_distance / np.float(dt)

		if encoder_velocity < 0.03:
			self.prev_time = time
			self.prev_ticks = fixed_inputs[1]
			self.counter += 1
			dt = 0
			return self.prev_pose

		# Update state-transition Jacobian
		Fk, Hk = self.update_model(dt)

		# Check which estimation method to use
		if self.estimation_method == None and self.filter == None:				# Dead-Reckoning
			inputs = np.array(fixed_inputs)
			# Modify state vector to reflect the current encoder readings
			states = np.array(self.prev_states)
			states[3] = encoder_velocity*np.cos(inputs[0])
			states[4] = encoder_velocity*np.sin(inputs[0])/self.r_dist
			states[5] = encoder_velocity
			states[6] = inputs[0]
			estimates = Fk.dot(states)
			# print("		Using Dead-Reckoning Estimation: ")
		else:
			alpha = fixed_inputs[0]
			gyro_enc = encoder_velocity * np.tan(alpha) / self.r_dist
			gyro_imu = fixed_inputs[2]
			v = encoder_velocity*np.cos(alpha)
			inputs = np.array([ [alpha, encoder_velocity, v, gyro_enc, gyro_imu] ])
			estimates,_ = self.filter.update(dt, inputs, self)

		# Extract pose-related variables
		pred_pose = [np.float(estimates[0]), np.float(estimates[1]), np.float(estimates[3])]

		# Store Latest variables for next step's calculations
		self.prev_time = time
		self.prev_steering_angle = fixed_inputs[0]
		self.prev_ticks = fixed_inputs[1]
		self.prev_pose = np.array(pred_pose)
		self.prev_states = estimates
		self.counter += 1
		# Debugging features
		if verbose == True:
			# print("--------- Estimate Update #" + str(self.counter-1) + " ---------")
			print("	Raw Inputs: " + str(raw_inputs))
			print("	Filtered Inputs: " + str(fixed_inputs))
			print("	Estimation Used Inputs: " + str(inputs))
			print("	Estimated Pose: " + str(pred_pose))
			# print("	State-Transition Jacobian [F]:\r\n" + str(Fk))

		return np.array(pred_pose)

	def update_model(self, dt):
		"""
		:brief: Update the state-transition model
			# States:
			x(t) =   |  x(t-1)	| [0]
					 |  y(t-1)	| [1]
					 |  θ(t-1)	| [2]
					 |  v(t-1)	| [3]
					 |  ω(t-1)	| [4]
				   	 |  vs(t-1)	| [5]
					 |  α(t-1)	| [6]

			# Time-continuous Process Model
			f(x,t) = |   x(t-1) + v(t-1)*dt*cos(θ(t-1))	|
					 |   y(t-1) + v(t-1)*dt*sin(θ(t-1))	|
		 			 | 		 θ(t-1) + ω(t-1)*dt			|
					 |       vs(t-1)*cos(α(t-1))		|
					 |       v(t-1)*tan(α(t-1)) / L		|
				   	 |              vs(t-1)				|
					 |              α(t-1)				|

		:param dt:
			Change in time [seconds] since last model update
		:return Fx:
			The updated model's state-transition matrix Jacobian
		"""

		# Previous
		x,y,theta,v,w,vs,a = self.prev_states

		# Intermediate values
		F03 = -v*dt*np.sin(theta); 			F04 = dt*np.cos(theta)
		F13 = v*dt*np.cos(theta);			F14 = dt*np.sin(theta)
		F24 = (dt*np.tan(a))/self.r_dist; 	F27 = v*dt/(np.power(np.cos(a),2)*self.r_dist)
		F36 = np.cos(a); 					F37 = -vs*np.sin(a)
		F46 = np.sin(a)/self.r_dist;		F47 = vs*np.cos(a)/self.r_dist

		# Time-continuous non-linear state transition Jacobian: F(x)
		Fx = np.array([ [1, 0, F03, F04, 0,  0,    0],
						[0, 1, F13, F14, 0,  0,    0],
						[0, 0,   1,   0, 1,  0,    0],
						[0, 0,   0,   0, 0, F36, F37],
						[0, 0,   0, F24, 0,   0, F27],
						[0, 0,   0,   0, 0,   1,   0],
						[0, 0,   0,   0, 0,   0,   1]
					])
		# Fx = np.array([ [1, 0, F03, F04, 0,  0,    0],
		# 				[0, 1, F13, F14, 0,  0,    0],
		# 				[0, 0,   1, F24, 0,  0,  F27],
		# 				[0, 0,   0,   0, 0, F36, F37],
		# 				[0, 0,   0,   0, 0, F46, F47],
		# 				[0, 0,   0,   0, 0,   1,   0],
		# 				[0, 0,   0,   0, 0,   0,   1]
		# 			])

		H20 = np.cos(a); 					H21 = -vs*np.sin(a)
		H30 = (dt*np.tan(a))/self.r_dist; 	H31 = v*dt/(np.power(np.cos(a),2)*self.r_dist)

		Hx = np.array([  [0, 0, 0,   0, 0,  0,    1],
						 [0, 0, 0,   0, 0,  1,    0],
						 [0, 0, 0,   0, 0, H20, H21],
						 [0, 0, 0, H30, 0,  0,  H31],
						 [0, 0, 0,   0, 1,  0,    0]
			 ])

		self.F = Fx
		self.H = Hx
		return Fx, Hx
