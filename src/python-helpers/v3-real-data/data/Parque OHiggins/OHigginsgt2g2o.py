# imports
import shlex
from math import *
import numpy as np

    
# filenames
gtLandmarks = "ground_truth_map.txt"
outG2O = "gt.g2o"
fg2o = open(outG2O, 'w')
landmarks = np.loadtxt(gtLandmarks, comments="#")

# transformation parameters
xtrans = 19.4
ytrans = -41.5
theta = (77.9*2*pi)/360

for i in range(landmarks.shape[0]):
    newLandmarkX = (landmarks[i,0]+xtrans)*cos(theta) - (landmarks[i,1]+ytrans)*sin(theta)
    newLandmarkY = (landmarks[i,0]+xtrans)*sin(theta) + (landmarks[i,1]+ytrans)*cos(theta)
    fg2o.write("VERTEX_XY " + str(i) + " " + str(newLandmarkX) + " " + str(newLandmarkY) + "\n")
