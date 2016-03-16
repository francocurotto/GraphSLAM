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

# imports
import subprocess
import sys
import time
sys.path.append('../commons')
from slamFunctions import *
from g2o2lab import *

# variables
g2oIterations = 20
nlandmarks = 40
simSteps = 300
infoOdomPos = 1000
infoOdomAng = 1000
infoPointSen = 1000
kernelWidth = 0.1

# run g2o tests
start_time = time.time()

# paths and filenames
binSimPath = "../../graphSLAM/build/"
suffix = "_it_" + str(g2oIterations)  + "_nl_" + str(nlandmarks) + "_op_" + str(infoOdomPos) + "_oa_" + str(infoOdomAng) + "_lp_" + str(infoPointSen) + "_ds_" + str(simSteps) + "_kw_" + str(kernelWidth)
simPath = "data/groundtruth" + suffix + ".g2o"
guessInPath = "data/initialGuessIn" + suffix + ".g2o"
guessOutPath = "data/initialGuessOut" + suffix + ".g2o"
optPath = "res/optimized" + suffix + ".g2o"
figPath = "res/res"

# get simulation data
subprocess.call([binSimPath+"./my_simulator",
                 "-hasOdom", "-hasPointSensor",
                 "-nlandmarks", str(nlandmarks), "-simSteps", str(simSteps),
                 "-infoOdomPos", str(infoOdomPos), "-infoOdomAng", str(infoOdomAng),
                 "-infoPointSen", str(infoPointSen), "-fixFirst", "-guessFilename", guessInPath,
                 simPath])

# get initial guess
subprocess.call(["g2o", "-i", "0", "-guessOdometry",
                 "-o", guessOutPath, guessInPath])

# make optimization
subprocess.call(["g2o", "-i", str(g2oIterations), "-guessOdometry",
                 #"-inc",
                 "-robustKernel", "Huber",
                "-robustKernelWidth", str(kernelWidth),
                 #"-solver",
                 "-o", optPath, guessInPath])

# compute elapsed time
elapsed_time = time.time() - start_time
print "Total time tests: " + str(elapsed_time) + " [s]"

# get results in lab format
g2o2lab(guessOutPath, optPath, "res_lab/")

# plot results
#plotGuess(simPath, guessOutPath, "res/guess"+ suffix)
plotResults(simPath, guessOutPath, optPath, figPath, suffix)
