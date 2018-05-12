import numpy as np

class TricycleModel(object):
	""" A Tricycle-model of a mobile platform, used to estimate the 2D pose
	(X, Y, heading) from the available sensors (namely encoders and gyro). """

    def __init__(self):
		""" Sets up all internally used variables and loads initial parameters """

		# Sensor Fusion Method (default None. Should be specified  from public call)
		self.fuser = None

		# Initial Variables
		self.x = 0
		self.y = 0
		self.heading = 0
		self.pose = [0, 0, 0]
		self.prev_states = [0, 0, 0, 0, 0]
		self.prev_time = 0
		self.prev_ticks = 0
		self.prev_steering_angle = 0

		# Physical Parameters and Constraints
		#    (TODO: add functions for easier externally user-defined constraints)
		self.steering_angle_limit = np.pi / 2.0 # Defines +/- steering angle limit in radians
		self.r_dist = 1.0 		# Distance b/w front and rear axis (meters)
		self.d_dist = 0.75		# Distance b/w rear wheels (meters)
		self.rear_radius = 0.2  # (meters)
		self.front_radius = 0.2 # (meters)
		self.ticks_per_rev = 512

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

	def process_inputs(self, raw_inputs, verbose=False):
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
		_time, steer_ang, enc_revs, gyro_z = raw_inputs

		# Check if steering angle is outside our working limits regardless of input (Safety)
		if steer_ang < -self.steering_angle_limit:
			steer_ang = -self.steering_angle_limit
		elif steer_ang > self.steering_angle_limit:
			steer_ang = self.steering_angle_limit

		# Check if we should be expecting the rest of the inputs to be preprocessed
		if self.expect_processed_inputs == True:
			processed_inputs = [_time, steer_ang, enc_revs, gyro_z]
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
			processed_inputs = [_time, steer_ang, enc_revs, gyro_z]

		# Debugging feature
		if verbose == True:
			print("Input Pre-Processing:")
			print("	Inputs Before Processing: " + str(raw_inputs))
			print("	Inputs After Processing: " + str(processed_inputs))

		return processed_inputs

	def estimate(self,time,steering_angle,encoder_ticks,angular_velocity, verbose=False):
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
		# Initialize internally used variables
		pose_hat = [0, 0, 0]
		# EKF-expected observation format: obs = [velocity_enc, heading_enc, heading_gyro]
		fuser_inputs = []

		# Derive conversion factor to convert ticks to meters
		ticks_per_meter = (2 * np.pi) / self.ticks_per_rev
		if self.encoder_on_front == True:
			ticks_per_meter = ticks_per_meter * self.front_radius
		else:
			ticks_per_meter = ticks_per_meter * self.rear_radius

		## Derive incrementally changing variables
		# Calculate time difference (sec) from last step
		dt = time - self.prev_time
		# Calculate the incremental encoder difference (ticks) from last step
		denc = encoder_ticks - self.prev_ticks
		# Calculate the change in linear displacement (meters) from last step
		d_distance = denc * ticks_per_meter

		# Combine function inputs into Sensor Fuser-generic form
		encoder_velocity = d_distance / float(dt)
		encoder_heading = d_distance / float(dt)


		# Store Latest variables for next step's calculations
		self.prev_time = time
		self.prev_ticks = encoder_ticks
		self.prev_steering_angle = steering_angle

		# Debugging features
		if verbose == True:
			print("Estimation Process: ")

		return pose_hat

		def kinematics_model(self):
			"""
			:brief
				The purpose of this function is to generate the time-continuous
				state-transition matrix and Jacobian useful for state-estimation.
			:return:
				Saturated inputs of same format as raw_inputs, and [secs, radians, ticks, rads/sec] units respectively
			"""

			[x_k        + v_k-1*dt*cos(theta_k-1), #update state variables
			 y_k        + v_k-1*dt*sin(theta_k-1),
			 theta_k    + w_k-1*dt,
			 v_k		+ v_str_k-1,
			 w_k]
