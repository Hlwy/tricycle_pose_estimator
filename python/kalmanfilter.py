import numpy as np

class KalmanFilter(object):
	""" A simple Extended Kalman Filter (EKF) that estimates the 2D-pose of a
	model, using wheel encoders and gyroscope data. The state variables are
	x, y, theta, v, w and the sensor measurements are TODO"""

	def __init__(self):
		""" Sets up the initial parameters """
		# initial beliefs
		self.x = 0
		self.y = 0
		self.theta = 0 #This is equivalent to yaw

		#Initialize state variables (also called xk) - x_0, y_0, theta_0, v_0, w_0
		self.states = np.array([self.x, self.y, self.theta, 0, 0])

		#noise in estimate mu, (also known as pk, the prediction error)
		self.sigma_sq = np.array([	[0.1, 0, 0, 0, 0],
									[0, 0.1, 0, 0, 0],
									[0, 0, 0.1, 0, 0],
									[0, 0, 0, 0.1, 0],
									[0, 0, 0, 0, 0.1]])

		#Initializing initial odom/imu values
		self.v_odom = 0
		self.w_odom = 0
		self.w_imu = 0
		self.z_t = np.array([self.v_odom, self.w_odom, self.w_imu])

		# update step noise (also known as Q)
		self.sigma_m_sq = np.array([[0.1, 0, 0, 0, 0],
									[0, 0.1, 0, 0, 0],
									[0, 0, 0.1, 0, 0],
									[0, 0, 0, 0.1, 0],
									[0, 0, 0, 0, 0.1]])
		# sensor noise
		self.sigma_z_sq = np.array([[0.1, 0, 0], #noise in v_odom
									[0, 0.5, 0], #noise in w_odom
									[0, 0, 0.02]]) #noise in w_gyro

		self.H = np.array([	[0, 0, 0, 1, 0],
							[0, 0, 0, 0, 1],
							[0, 0, 0, 0, 1]])

	def step(self, dt, model):

		# Do Kalman updates
		curr_time = rospy.Time.now()
		dt = (curr_time - last_time)
		last_time = curr_time

		x_k, y_k, theta_k, v_k, w_k = self.states
		# print self.z_t
		self.z_t = np.array([self.v_odom, self.w_odom, self.w_imu])
		#Predict step
		self.states = np.array([x_k + v_k*dt*cos(theta_k), #update state variables
								y_k + v_k*dt*sin(theta_k),
									theta_k + w_k*dt,
										v_k,
										w_k])

		#The Jacobian of update model
		F_k = np.array([[1, 0, -v_k*dt*sin(theta_k), dt*cos(theta_k), 0],
						[0, 1, -v_k*dt*cos(theta_k), dt*sin(theta_k), 0],
						[0, 0, 1, 0, dt],
						[0, 0, 0, 1, 0],
						[0, 0, 0, 0, 1]])

		#update error in prediction
		self.sigma_sq = F_k.dot(self.sigma_sq).dot(F_k.T) + self.sigma_m_sq

		#Update step
		measurement_residual = self.z_t - self.H.dot(self.states)
		residual_covariance = self.H.dot(self.sigma_sq).dot(self.H.T) + self.sigma_z_sq
		K_t = self.sigma_sq.dot(self.H.T).dot(np.linalg.inv(residual_covariance)) #Kalman gain

		self.states = self.states + K_t.dot(measurement_residual)
		self.sigma_sq = (np.eye(len(self.states))-K_t.dot(self.H)).dot(self.sigma_sq)
