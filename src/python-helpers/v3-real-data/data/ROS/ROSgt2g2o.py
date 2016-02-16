# imports
import shlex
import math
import numpy as np
import yaml

    
# filenames
gtPose = "gtpose.dat"
gtLandmarks = "ground_truth_sim_art.yaml"
outG2O = "gt.g2o"
fg2o = open(outG2O, 'w')

poses = np.loadtxt(gtPose, comments="#")

for i in range(poses.shape[0]):
    fg2o.write("VERTEX_SE2 " + str(i) + " " + str(poses[i,2]) + " " + str(poses[i,3]) + " " + str(poses[i,4]) + "\n")
    
with open(gtLandmarks, 'r') as stream:
    landmarks = yaml.load(stream)
    
for l in landmarks["ground_truth/trees"]:
    i += 1
    fg2o.write("VERTEX_XY " + str(i) + " " + str(l[0]) + " " + str(l[1]) + "\n")
    
fg2o.close()


