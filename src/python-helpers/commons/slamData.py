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
import shlex
import string

# slam data class
class slamData():
    """
    SLAM data container
    
    Attributes
    ---------
    landmarkX: array
               Array of landmarks x position
    landmarkY: array
               Array of landmarks y position 
    poseX: array
           Array of poses x position
    poseY: array
           Array of poses y position  
    poseA: array
           Array of poses orentation (angle)
    """
    def __init__(self, dataFilename):
        self.landmarkX = []
        self.landmarkY = []
        self.poseX = []
        self.poseY = []
        self.poseA = []
        self.getDataFromFile(dataFilename)
        
    def getDataFromFile(self, dataFilename):
        """
        Fills object arrays with SLAM data given by file
        
        Parameters
        ----------
        dataFilename: string
                      filename with SLAM data in g2o format
        """
        f = open(dataFilename, 'r')
        # get data loop
        for line in f:
            # split string
            lineWords = shlex.split(line)
            if string.find(lineWords[0], "VERTEX_SE2") != -1:
                # get robot pose
                self.poseX.append(float(lineWords[2]))
                self.poseY.append(float(lineWords[3]))
                self.poseA.append(float(lineWords[4]))
            elif string.find(lineWords[0], "VERTEX_XY") != -1:
                # get landmark position
                self.landmarkX.append(float(lineWords[2]))
                self.landmarkY.append(float(lineWords[3]))
        f.close()
