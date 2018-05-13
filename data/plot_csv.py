import matplotlib.pyplot as plt
from csv import reader
from dateutil import parser
from matplotlib import animation
import math
import numpy as np

file1 = 'trike_straight'
file2 = 'trike_circle'

fp1 = "processed/" + str(file1) + "_extracted_truth.csv"
fp2 = "processed/" + str(file2) + "_extracted_truth.csv"

with open(fp1, 'r') as fIn:
    straightTruth = list(reader(fIn))

stX = [i[0] for i in straightTruth[1::]]
stY = [i[1] for i in straightTruth[1::]]
stYaw = [i[2] for i in straightTruth[1::]]

with open(fp2, 'r') as f2In:
    circleTruth = list(reader(f2In))

cX = [i[0] for i in circleTruth[1::]]
cY = [i[1] for i in circleTruth[1::]]
cYaw = [i[2] for i in circleTruth[1::]]

fig = plt.figure()
line = fig.add_subplot(2,1,1)
err = fig.add_subplot(2,1,2)

line.clear()
line.plot(stY, stX,'g')
# line.plot(cY, cX,'g',acty,actx,'b',refy,refx,'g')
line.set_title('Position')
line.set_xlabel('X (m)')
line.set_ylabel('Y (m)')
line.axis('equal')
line.autoscale()

err.clear()
err.plot(cY, cX,'g')
err.set_title('Position')
err.set_xlabel('X (m)')
err.set_ylabel('Y (m)')
err.axis('equal')
err.autoscale()



plt.show()
