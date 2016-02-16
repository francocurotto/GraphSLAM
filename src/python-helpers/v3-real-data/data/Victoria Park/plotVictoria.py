# imports
import shlex
import math
import matplotlib.pyplot as plt
import sys

inDeadReckon = "deadReckoning.dat"
inMeasurement = "measurements.dat"

odometry = [line.rstrip('\n') for line in open(inDeadReckon)]
del odometry[0]
measurements = [line.rstrip('\n') for line in open(inMeasurement)]
del measurements[0]

poseX = []
poseY = []
landX = []
landY = []
j = 0


nPoses = 500
for i in range(nPoses):
    #print "pose: " + str(i)
    odomLine = odometry[i]
    odomWords = shlex.split(odomLine)
    x1 = float(odomWords[1])
    y1 = float(odomWords[2])
    a1 = float(odomWords[3])
    poseX.append(x1)
    poseY.append(y1)
    
    odomTime = float(odomWords[0])
    while j < len(measurements):
        #print "land: " + str(j)
        measLine = measurements[j]
        measWords = shlex.split(measLine)
        measTime = float(measWords[0])
        prevTime = 0
        nextTime = float("inf")
        if i != 0:
            prevLine = odometry[i-1]
            prevWords = shlex.split(prevLine)
            prevTime = float(prevWords[0])
        if i != nPoses:
            nextLine = odometry[i+1]
            nextWords = shlex.split(nextLine)
            nextTime = float(nextWords[0])
        if measTime > ((odomTime + nextTime)/2):
            break
        if measTime >= ((odomTime + prevTime)/2):
            mr = float(measWords[1])
            mt = float(measWords[2])
            lx = mr*math.cos(mt+a1 - math.pi/2) + x1
            ly = mr*math.sin(mt+a1 - math.pi/2) + y1
            landX.append(lx)
            landY.append(ly)
        j = j+1
        
lw = 1
ms = 4
f = plt.figure();
plt.plot(landX, landY, 'r+', linewidth = lw, markersize = ms, label='Guess landmarks')
plt.plot(poseX, poseY, 'b-', linewidth = lw, markersize = ms, label='Odometry path')
plt.grid(True)
ax = plt.gca()
ax.relim()
ax.autoscale_view()
plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
plt.savefig("victoriaPlot.pdf", bbox_inches='tight')
plt.show()
        
