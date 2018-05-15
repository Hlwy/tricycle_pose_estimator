import numpy as np

class EKF(object):
	""" A simple Extended Kalman Filter (EKF) used for estimating the pose of a
	mobile base using the available sensor observations. """

	def __init__(self, n_states, n_observations):
		""" Sets up the initial parameters """
		# Name used for external identification
		self.name = "Estimation Method [Extended Kalman Filter]"
		print("Initializing " + str(self.name))

		# Initial pose estimates
		self.n_updates = 0		# Counter for # of 'update' function calls
		self.N = n_states		# Number of states
		self.M = n_observations	# Number of sensor observations

		# Initialize internal parameters that need to be defined by externally linked model.
		self.X = np.zeros((self.N,1)) 		# State Matrix                      [N x 1]
		self.F = np.zeros((self.N,self.N)) 	# Process Model Jacobian            [N x N]
		self.H = np.zeros((self.M,self.N)) 	# Measurement Model Jacobian        [M x N]
		self.P = np.zeros((self.N,self.N)) 	# Predicted State Error Covariance  [N x N]
		self.R = np.identity(self.M) 		# Measurement Error Covariance      [M x M]
		self.Q = np.identity(self.N) 		# Process Noise Covariance          [N x N]
		self.I = np.identity(self.N) 		# Identity Matrix                   [N x N]
		self.Z = np.zeros((self.M,1))		# Sensor Observation Matrix 		[M x 1]

	def setX(self, x):
		if x.shape == self.X.shape:
			self.X = x
		else:
			print("ERROR: The State Matrix [X] must be of size [N x 1]")

	def setP(self, p):
		if p.shape == self.P.shape:
			self.P = p
		else:
			print("ERROR: The Predicted State Error Covariance Matrix [P] must be of size [N x N]")

	def setF(self, f):
		if f.shape == self.F.shape:
			self.F = f
		else:
			print("ERROR: The Measurement Model Jacobian Matrix [F] must be of size [N x N]")

	def setH(self, h):
		if h.shape == self.H.shape:
			self.H = h
		else:
			print("ERROR: The Process Model Jacobian Matrix [H] must be of size [M x N]")

	def setQ(self, q):
		# print("Q Matrix (before setting):\r\n" + str(self.Q))
		if q.shape[1] == self.Q.shape[0]:
			self.Q = np.multiply(np.identity(self.N), q.T)
		else:
			print("ERROR: The Process Noise Covariance Matrix [Q] must be of size [N x N]")
		# print("Q Matrix (after setting):\r\n" + str(self.Q))

	def setR(self, r):
		# print("R Matrix (before setting):\r\n" + str(self.R))
		if r.shape[1] == self.R.shape[0]:
			self.R = np.multiply(np.identity(self.M), r.T)
		else:
			print("ERROR: The Measurement Error Covariance Matrix [R] must be of size [M x M]")
		# print("R Matrix (after setting):\r\n" + str(self.R))

	def update(self, dt, inputs, model=None, debug=True):
		# Modify variable naming scheme for easier user reading and typing
		old_x = self.X			# Previous time-step states
		old_p = self.P			# Previous time-step error covariance
		obs = np.array(inputs)	# Current time-step sensor observations
		zk = []
		# Just name contractions
		R = self.R; Q = self.Q; I = self.I

		# Check matrix sizes
		if obs.shape == self.Z.shape:
			zk = obs
		elif obs.T.shape == self.Z.shape:
			zk = obs.T
		else:
			print("ERROR: EKF observation matrix size wrong!")

		# Use internally stored filter matrices if no model is defined
		if model == None:
			F = self.F
			H = self.H
			# print("No Model Specified.")
		else: # Figure out a more generalizable method for EKF <-> model inter-operability
			F = np.array(model.F)
			H = np.array(model.H)
			# old_x = model.prev_states
			# print("Model [" + str(model.name) + "] Specified.")

		# Generate some random noise
		# state_noise = np.random.normal(np.zeros_like(old_x),scale=0.0025)
		state_noise = np.random.normal(np.array([[0.00001],[0.00001],[0.00000001],[0.01],[0.01],[0.01],[0.00001]]),scale=0.002)
		# state_noise = np.random.normal(np.zeros_like(old_x),scale=0)

		"""
		Prediction Stage
			xhat = F * xhat + wk
			Phat = F * Phat * trans(F) + Q
		"""
		pred_x = F.dot(old_x) + state_noise
		pred_p = F.dot(old_p).dot(F.T) + Q

		"""
		Update Stage
			residual = data - H * xhat			  # Measurement Residual
			S = H * Phat * trans(H) + R 		  # Update Innovation Covariance
			K = Phat * trans(H) * inv(S)		  # Kalman Gain

			xhat = xhat + K * residual;
			Phat = (In - K * H) * Phat;
		"""
		measurement_residual = zk - (H.dot(pred_x))
		residual_covariance = H.dot(pred_p).dot(H.T) + R
		kalman_gain = pred_p.dot(H.T).dot(np.linalg.inv(residual_covariance))

		new_x = pred_x + kalman_gain.dot(measurement_residual)
		new_p = (I - kalman_gain.dot(H)).dot(pred_p)

		if debug == True:
			# print("Matrix Sizes:")
			# print("	Inputs Shape: " + str(zk.shape))
			# print("	Residual Shape: " + str(measurement_residual.shape))
			# print("	Innovation [S] Shape: " + str(residual_covariance.shape))
			# print("	K Shape: " + str(kalman_gain.shape))
			print("Matrices:")
			# print("	F:\r\n" + str(F) + "\r\n")
			# print("	H:\r\n" + str(H) + "\r\n")
			print("	Inputs:\r\n" + str(zk) + "\r\n")
			print("	Predicted X:\r\n" + str(pred_x) + "\r\n")
			print("	Predicted P:\r\n" + str(pred_p) + "\r\n")
			print("	Residual:\r\n" + str(measurement_residual) + "\r\n")
			print("	Residual Covariance:\r\n" + str(residual_covariance) + "\r\n")
			# print("	K:\r\n" + str(kalman_gain) + "\r\n")
			print("	Updated States [X]:\r\n" + str(new_x) + "\r\n")


		# Store locally for safe-keeping and return output
		self.X = new_x
		self.P = new_p
		self.n_updates += 1
		return new_x, new_p
