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
 * Data association functions implementation.
 */ 

#include <iostream>
#include <Eigen/Core>
#include <limits.h>
#include <algorithm>

#include "g2o/core/sparse_optimizer.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

/**
 * Performs one iteration of the incremental data association algorithm in a SLAM graph.
 * 
 * Tests association of the current observed landmarks (obverved in the current pose), and all the past
 * observed landmarks. Merges landmarks that pass the test. It also uses a distance test to speed up
 * the algorithm.
 * @param optimizer SparseOptimizer object that contains the SLAM graph
 * @param poseIndex Index of the current pose (in the VertexContainer vector obtained from the optimizer)
 * @param xi Threshold for the correspondence test
 * @param maxDistance Maximum distance allowed between landmarks to produce an association
 * @see correspondenceTest()
 * @see distanceTest()
 * @return True: no association found, sFalse: at least one association found
 */ 
bool incDataAssociation (SparseOptimizer& optimizer, int poseIndex, double xi, double maxDistance);

/**
 * Performs the full data association algorithm in a SLAM graph.
 * 
 * Tests association between all possible pairs of landmarks, up to the ones observed in the current pose.
 * Merges landmarks that pass the test. Uses the incDataAssociation() function iteratively.  
 * @param optimizer SparseOptimizer object that contains the SLAM graph
 * @param poseIndex Index of the current pose (in the VertexContainer vector obtained from the optimizer)
 * @param xi Threshold for the correspondence test
 * @param maxDistance Maximum distance allowed between landmarks to produce an association
 * @see incDataAssociation()
 * @return True: no association found, False: at least one association found
 */
bool fullDataAssociation (SparseOptimizer& optimizer, int poseIndex, double xi, double maxDistance);

/**
 * Tests if two landmarks correspond to the same using a likelihood test.
 * 
 * Computes the marginal covariance matrix using a g2o function, then checks if the probability of the two landmarks 
 * being in the same position is greater than a threshold xi.
 * @param optimizer SparseOptimizer object that contains the SLAM graph
 * @param v1 First landmark (vertex) under test
 * @param v2 Second landmark (vertex) under test
 * @param xi Threshold for the correspondence test, the greater the threshold, the more strict the test
 * @return True: the pair pass the test (they correspond to the same landmark)
 */
bool correspondenceTest (SparseOptimizer& optimizer, OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double xi);

/**
 * Tests if the distance between two landmarks is shorter than a given maximum distance.
 * @param v1 First landmark (vertex) under test
 * @param v2 Second landmark (vertex) under test
 * @param maxDistance Maximum distance for the landmarks to pass the test
 * @return True: the pair pass the test
 */
bool distanceTest (OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2, double maxDistance);
