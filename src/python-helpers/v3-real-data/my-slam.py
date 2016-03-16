#!/usr/bin/python

'''
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

#imports
import subprocess
import sys
import time
import os
from time import gmtime, strftime
sys.path.append('../commons')
sys.path.append('data/ROS')
sys.path.append('data/Parque OHiggins')
sys.path.append('data/Victoria Park')
from slamFunctions import *
from g2o2lab import *
from ROSRaw2g2o import *
from OHigginsRaw2g2o import *
from victoriaRaw2g2o import *
    
# variables
g2oIterations = 10
xi = 0
infoOdomPos = 100000
infoOdomAng = 100000
infoPointSen = 100
dataSkip = 1
interOpt = 500
dataSize = 100000
disTest = 3
kernelWidth = 1
poseSkip = 10

# compile
buildPath = "../../graphSLAM/build/"
subprocess.call(["make", "-C", buildPath]) 

# run g2o tests
start_time = time.time()

# paths
#dataPath = "data/ROS/ROS.g2o"
dataPath = "data/Parque OHiggins/ohiggins.g2o"
#dataPath = "data/Victoria Park/victoria.g2o"
dataName = os.path.splitext(os.path.basename(dataPath))[0]
dataDir = os.path.dirname(dataPath) + "/"
gtPath = dataDir + "gt.g2o"
guessOutPath = "res/initialGuessOut_" + dataName + ".g2o"
resPath = "res/optimized_" + dataName + ".g2o"
figPath = "res/res_" + dataName

# generate g2o format data
print "Generating data in g2o format"
if dataName == "ROS":
    ROSRaw2g2o(infoOdomPos, infoOdomAng, infoPointSen, dataDir, dataSkip, dataSize)
if dataName == "ohiggins":
    ohigginsRaw2g2o(infoOdomPos, infoOdomAng, infoPointSen, dataDir, dataSkip, dataSize)
elif dataName == "victoria":
    victoriaRaw2g2o(infoOdomPos, infoOdomAng, infoPointSen, dataDir, dataSkip, dataSize)


# get initial guess
subprocess.call(["g2o", "-i", "0", "-guessOdometry",
                 "-o", guessOutPath, dataPath])
                 
# optimize
print "Optimize"
subprocess.call(["env", "CPUPROFILE=./my_slam_prof.prof",
                buildPath+"./my_slam", 
                "-i", str(g2oIterations), 
                "-t", str(xi),
                "-robustKernel", "Huber",
                "-robustKernelWidth", str(kernelWidth),
                "-poseSkip", str(poseSkip),
                "-interOpt", str(interOpt),
                "-disTest", str(disTest),
                "-o", resPath, guessOutPath])
    
# get results in lab format
g2o2lab(guessOutPath, resPath, "res_lab/")

# plot results
currTime = strftime("_%Y-%m-%d %H:%M:%S", gmtime())
suffix = "_it_" + str(g2oIterations)  + "_xi_" + str(xi) + "_op_" + str(infoOdomPos) + "_oa_" + str(infoOdomAng) + "_lp_" + str(infoPointSen) + "_dsk_" + str(dataSkip) + "_io_"  + str(interOpt) + "_ds_" + str(dataSize) + "_dt_" + str(disTest) + "_kw_" + str(kernelWidth) + "_ps_" + str(poseSkip)
plotResults(gtPath, guessOutPath, resPath, figPath, currTime + suffix)

# compute elapsed time
elapsed_time = time.time() - start_time
print "Total time tests: " + str(elapsed_time) + " [s]"
