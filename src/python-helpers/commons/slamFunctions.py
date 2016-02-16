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
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import math
from slamData import *
        
# plots functions
def plotGuess (gtFilename, guessFilename, figFilename):
    """
    Plots groundtruth and initial guess of SLAM problem.
    
    Parameters
    ----------
    gtFilename: string
                path for groundtruth data
    guessFilename: string
                   path for initial guess data
    figFilename: string
                 path for output figure (without extension) 
    """
    
    # get variables from file
    gtData = slamData(gtFilename)
    guessData = slamData(guessFilename)
    
    lw = 3
    ms = 8
    
    if gtData is not None:
        gtRobPlt, = plt.plot(gtData.poseX, gtData.poseY, '.-', color = '#bbbbf9', linewidth = lw, markersize = ms, label='GT robot path')
    lanPlt, = plt.plot(guessData.landmarkX, guessData.landmarkY, 'r+', linewidth = lw, markersize = ms, label='landmarks')
    robPlt, = plt.plot(guessData.poseX, guessData.poseY, 'b-', linewidth = lw, markersize = ms, label='robot path')
    if gtData is not None:
        gtLanPlt, = plt.plot(gtData.landmarkX, gtData.landmarkY, '.', color = '#800000', linewidth = lw, markersize = ms, label='GT landmarks')
    
    ax = plt.axes()
    ax.grid(True)
    ax.set_title("Initial Guess")
    ax.relim()
    ax.autoscale_view()
    ax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.savefig(figFilename + ".pdf", bbox_inches='tight')
    
def plotResults(gtFilename, guessFilename, optFilename, figFilename, suffix=""):
    """
    Plots groundtruth, initial guess, and optimized results of SLAM (simulated) problem.
    
    Parameters
    ----------
    gtFilename: string
                path for groundtruth data
    guessFilename: string
                   path for initial guess data
     optFilename: string
                  path for optimized result data
    figFilename: string
                 path for output figure (without extension) 
    suffix: string, optional
            string to append in figure's name
    """
    
    # get variables from file
    gtData = slamData(gtFilename)
    guessData = slamData(guessFilename)
    optData = slamData(optFilename)
    
    # create figure
    f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
    makeSubplot(ax1, gtData, guessData, "Initial Guess", False)
    makeSubplot(ax2, gtData, optData, "After Solver", True)
    #plt.show()
    
    # print figure
    #plt.savefig(figFilename + ".png", bbox_inches='tight')
    plt.savefig(figFilename + suffix + ".pdf", bbox_inches='tight')
    
    # plot path error
    pathPlot(gtData, optData, figFilename, suffix)
    
def makeSubplot(ax, gtData, slamData, title, useLegend):
    """
    Plots two set of SLAM data in axis object.
    
    Parameters
    ----------
    gtData: slamData
            data for groundtruth
    slamData: slamData
              data from sover or initial guess
    title: string
           axis title
    useLegend: boolean
               True: add legend to axis
    """
    lw = 2
    ms = 8
    if gtData is not None:
        gtRobPlt, = ax.plot(gtData.poseX, gtData.poseY, '.-', color = '#bbbbf9', linewidth = lw, markersize = ms, label='GT robot path')
    lanPlt, = ax.plot(slamData.landmarkX, slamData.landmarkY, 'r+', linewidth = lw, markersize = ms, label='landmarks')
    robPlt, = ax.plot(slamData.poseX, slamData.poseY, 'b-', linewidth = lw, markersize = ms, label='robot path')
    if gtData is not None:
        gtLanPlt, = ax.plot(gtData.landmarkX, gtData.landmarkY, '.', color = '#800000', linewidth = lw, markersize = ms, label='GT landmarks')
    ax.grid(True)
    ax.set_title(title)
    ax.relim()
    ax.autoscale_view()
    if useLegend:
        ax.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
        
def pathPlot(gtData, optData, figFilename, suffix=""):
    """
    Plots path error of SLAM problem.
    
    Parameters
    ----------
    gtData: slamData
            data for groundtruth
    optData: slamData
             data for optimized result
    figFilename: string
                 path for output figure (without extension)
    suffix: string, optional
            string to append in figure's name
    """
    
    lw = 3
    pathError = []
    firstError = math.sqrt((gtData.poseX[0] - optData.poseX[0])**2 + 
    (gtData.poseY[0] - optData.poseY[0])**2)
    pathError.append(firstError)
    
    for i in range(1, len(gtData.poseX)):
        error = math.sqrt((gtData.poseX[i] - optData.poseX[i])**2 + 
        (gtData.poseY[i] - optData.poseY[i])**2)
        pathError.append(error + pathError[i-1]);
    
    for i in range(len(pathError)):
        pathError[i] = pathError[i] / (i+1);
    
    f = plt.figure();
    plt.plot(range(1,len(pathError)+1), pathError, linewidth = lw)
    plt.grid(True)
    f.suptitle("Path Error")
    plt.xlabel("Timestep")
    plt.ylabel("Normalized error")
    plt.xlim([1, len(pathError)+1])
    plt.ylim([0, max(pathError)])
    #plt.savefig(figFilename + "_path.png", bbox_inches='tight')
    plt.savefig(figFilename + suffix + "_path.pdf", bbox_inches='tight')
    
def makeRealPlots (guessPath, optPath, figPath, suffix=""):
    """
    Plots initial guess and optimized results of SLAM (real) problem.
    
    Parameters
    ----------
    guessFilename: string
                   path for initial guess data
     optFilename: string
                  path for optimized result data
    figFilename: string
                 path for output figure (without extension) 
    suffix: string, optional
            string to append in figure's name
    """
    # get data from file
    guessData = slamData(guessPath)
    optData = slamData(optPath)
    f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
    makeSubplot(ax1, None, guessData, "Initial Guess", False)
    makeSubplot(ax2, None, optData, "After Solver", True)
    
    # make figure
    plt.savefig(figPath + suffix + ".pdf", bbox_inches='tight')
    #plt.show()
