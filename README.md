# GraphSLAM

A GraphSLAM implementation using g2o framework

## Requirements

* CMake (https://cmake.org/)
* Eigen3 (http://eigen.tuxfamily.org/index.php?title=Main_Page)
* suitesparse (http://faculty.cse.tamu.edu/davis/suitesparse.html)
* g2o (https://github.com/RainerKuemmerle/g2o)

This project was developed in linux platform, with C++11 and Python 2.7 

## Directories:
- src:
    - graphSLAM: C++ scripts that performs SLAM magic
    - python-helpers: Python scripts that make easy to run GraphSLAM tests
- doc:
    - Documentation: Code documentation
    - Memoria: Memoria (tesis) document explaining the work
    
## Compilation

To compile the C++ scripts simply go to the `src/graphSLAM` folder and do:

- `mkdir build`
- `cd build`
- `cmake ..`
- `make`

Notice that you must have g2o installed in your machine.

## Execution

- For simulated data:
    1. Run the simulator binary `my-simulator` to generate the simulated data (use `-h` for options). 
    2. Run the GraphSLAM binary `my-slam` to optimize the data of the simulator. Use the "guess" file for the optimization (use `-h` for options).
    
    * Use `-anonymize` flag from `my-simulator` to anonymize landmarks (for unknown data association).
    * Alternatively run `g2o -i 0 -guessOdometry -o <guessInFile> <guessOutFile>` to generate an initial guess file (for plotting).
    * See `src/python-helpers/commons/` for functions to plot the results and change its format.

- For real data: just make sure that your data is in g2o format and proceed as before.

**See`src/python-helpers` subdirectories for test srcipts ready to run.**
