import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np

# file = '/home/hunter/theHuntingGround/src/tricycle_pose_estimator/data/processed/trike_straight_generated_outputs.csv'
file = '/home/hunter/theHuntingGround/src/tricycle_pose_estimator/data/processed/trike_circle_generated_outputs.csv'

outputs = np.genfromtxt(file, delimiter=',')
x = outputs[1:,0]
y = outputs[1:,1]
yaw = outputs[1:,3]

fig = plt.figure()
line = fig.add_subplot(2,1,1)
ang = fig.add_subplot(2,1,2)

def animate(i):
	global file
	outputs = np.genfromtxt(file, delimiter=',')

	x = outputs[1:,0]
	y = outputs[1:,1]
	yaw = outputs[1:,2]
	refX = outputs[1:,3]
	refY = -1 * outputs[1:,4]
	refYaw = outputs[1:,5]
	steps = range(yaw.shape[0])

	line.clear()
	line.plot(refY, refX,'g',y,x,'r')
	line.set_title('EKF Position')
	line.set_xlabel('X (m)')
	line.set_ylabel('Y (m)')
	line.axis('equal')
	# line.set_ylim([-5, 5])
	# line.set_xlim([-4, 0])
	# line.autoscale()

	ang.clear()
	ang.plot(steps, refYaw,'g', steps, yaw,'r')
	ang.set_title('EKF Yaw')
	ang.set_xlabel('Steps')
	ang.set_ylabel('Yaw (rad)')
	ang.set_ylim([-np.pi, np.pi])
	ang.autoscale()

	return line, ang,

anim = animation.FuncAnimation(fig, animate, interval=25)

plt.show()
