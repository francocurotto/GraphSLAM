/*
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
*/

/** @file 
 * Miscellaneous functions implementation.
 */ 

#include <iostream>
#include <string>
#include <fstream>
#include <Eigen/Core>

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/sparse_optimizer.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

/**
 * List the robust kernels available to use in the optimization stage.
 * @param list True to produce the list
 */
void listRobustKernels (bool list);

/**
 * Open file with the SLAM data.
 * @param ifs File object to open the data
 * @param filename Name of the file with the SLAM data in g2o format
 */
void readDataFile (ifstream &ifs, string filename);

/**
 * Write the result of the optimization in an output file, in the g2o format.
 * @param outputFilename Name of the file where the results will be written
 * @param optimizer SparseOptimizer object with the optimized SLAM graph
 */
void writeDataFile (string outputFilename, SparseOptimizer &optimizer);

/**
 * Load a robust kernel function on every node of the SLAM graph.
 * @param robustKernel String: use this robust error function
 * @param nonSequencial True: apply the robust kernel only on loop closures and not odometries
 * @param huberWidth Double: width for the robust Kernel (only if robustKernel) (default: -1)
 * @param optimizer SparseOptimizer object that contains the SLAM graph
 */
void loadRobustKernel (string robustKernel, bool nonSequential, double huberWidth, SparseOptimizer &optimizer);

/**
 * Fill an array with all the poses of the SLAM graph.
 * @param optimizer SparseOptimizer object that contains the SLAM graph
 * @param poses Array of poses to fill
 */
void getAllPoses (SparseOptimizer &optimizer, OptimizableGraph::VertexContainer &poses);

/**
 * Computes the theoretical maximum distance in which an associaction can be achieved,
 * given correspondence threshold xi.
 * @param xi Threshold for the correspondence test
 * @return Maximum distance
 */
double getMaxDistance (double xi);
