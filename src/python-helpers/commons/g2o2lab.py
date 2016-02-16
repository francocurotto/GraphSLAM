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

from slamFunctions import *
from slamData import slamData

def g2o2lab(guessPath, optPath, resDir):
    """
    Converts SLAM data from g2o format to alternative format (lab).
    
    Parameters
    ----------
    guessPath: string
               initial guess file in g2o format
    optPath: string
             optimized result file in g2o format
    resdir: directory to output leb format
    """
    
    resDir = "res_lab/"
    guessData = slamData(guessPath)
    optData = slamData(optPath)
    
    fd = open(resDir + 'deadReckoning.dat', 'w')
    for i in range(len(guessData.poseX)):
        fd.write(str(i) + " " + str(guessData.poseX[i]) + " " + str(guessData.poseY[i]) + " " + str(guessData.poseA[i]) + "\n")
    fd.close()

    fp = open(resDir + 'particlePose.dat', 'w')
    for i in range(len(optData.poseX)):
        fp.write(str(i) + " 0 " + str(optData.poseX[i]) + " " + str(optData.poseY[i]) + " " + str(optData.poseA[i]) + " 1 \n")
    fd.close()

    fl = open(resDir + "landmarkEst.dat", 'w')
    for i in range(len(optData.landmarkX)):
        fl.write(str(1) + " " + str(0) + " " + str(optData.landmarkX[i]) + " " + str(optData.landmarkY[i]) + " 1 0 1 1\n")
    fl.close()
