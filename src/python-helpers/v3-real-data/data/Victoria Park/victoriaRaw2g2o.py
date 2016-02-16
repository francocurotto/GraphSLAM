# imports
import shlex
import math

def victoriaRaw2g2o(infoOdomPos, infoOdomAng, infoPointSen, dataDir, dataSkip, dataSize):
    
    # filenames
    inDeadReckon = dataDir + "deadReckoning.dat"
    inMeasurement = dataDir + "measurements.dat"
    outG2O = dataDir + "victoria.g2o"
    fg2o = open(outG2O, 'w')
    
    odometry = [line.rstrip('\n') for line in open(inDeadReckon)]
    del odometry[0]
    measurements = [line.rstrip('\n') for line in open(inMeasurement)]
    del measurements[0]
    
    poseID = 0
    landID = len(odometry)
    j = 1
    count = 0
    for i in range((len(odometry))):
        odomLine = odometry[i]
        if count > dataSize:
            continue
        # data skip
        if poseID % dataSkip == 0:
            odomWords = shlex.split(odomLine)
            # check not last pose
            if i+dataSkip < len(odometry):
                # odometry
                nextWords = shlex.split(odometry[i+dataSkip])
                x1 = float(odomWords[1])
                y1 = float(odomWords[2])
                a1 = float(odomWords[3])
                x2 = float(nextWords[1])
                y2 = float(nextWords[2])
                a2 = float(nextWords[3])
                dx =  (x2 - x1)*math.cos(a1) + (y2 - y1)*math.sin(a1)
                dy = -(x2 - x1)*math.sin(a1) + (y2 - y1)*math.cos(a1)
                dt = ((a2 - a1 + math.pi) % (2*math.pi)) - math.pi
                fg2o.write("EDGE_SE2 " + str(poseID) + " " + str(poseID+dataSkip) + " " + str(dx) + " " +
                str(dy) + " " + str(dt) + " " + str(infoOdomPos) + " 0 0 " + str(infoOdomPos) + " 0 " + str(infoOdomAng) + "\n")
                count = count+1
            
            # measurements
            odomTime = float(odomWords[0])
            while j < len(measurements):
                measLine = measurements[j]
                measWords = shlex.split(measLine)
                measTime = float(measWords[0])
                prevTime = 0
                nextTime = float("inf")
                if i != 0:
                    prevLine = odometry[i-1]
                    prevWords = shlex.split(prevLine)
                    prevTime = float(prevWords[0])
                if i != len(odometry)-1:
                    nextLine = odometry[i+1]
                    nextWords = shlex.split(nextLine)
                    nextTime = float(nextWords[0])
                #print "odomTime: " + str(odomTime) + ", measTime : " + str(measTime) + ", prevTime: " + str(prevTime)
                if measTime > ((odomTime + nextTime)/2):
                    break
                if measTime >= ((odomTime + prevTime)/2):
                    mr = float(measWords[1])
                    mt = float(measWords[2])
                    lx = mr*math.cos(mt - math.pi/2)
                    ly = mr*math.sin(mt - math.pi/2)
                    fg2o.write("EDGE_SE2_XY " + str(poseID) + " " + str(landID) + " " + str(lx) +
                        " " + str(ly) + " " + str(infoPointSen) + " 0 " + str(infoPointSen) + "\n")
                    landID = landID + 1
                j = j+1
        poseID = poseID + 1
    
    fg2o.write("FIX " + str(0) + "\n")
    fg2o.close()
