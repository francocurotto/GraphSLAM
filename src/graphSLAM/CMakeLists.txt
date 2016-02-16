cmake_minimum_required(VERSION 2.8.3)
project(My_SLAM CXX)

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

# C++11 support
IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
endif()

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(G2O REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(CSparse)

INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse g2o_types_slam2d_addons)

# my-simulator build
ADD_LIBRARY(g2o_simulator_library ${G2O_LIB_TYPE}
   my-simulator/simulator.cpp my-simulator/simulator.h
   my-simulator/pointsensorparameters.cpp  my-simulator/pointsensorparameters.h
   my-simulator/simulator2d_base.h my-simulator/simulator2d.h
   my-simulator/sensor_odometry2d.cpp my-simulator/sensor_odometry2d.h 
   my-simulator/sensor_pose2d.cpp my-simulator/sensor_pose2d.h
   my-simulator/sensor_pointxy.cpp my-simulator/sensor_pointxy.h
   my-simulator/sensor_pointxy_bearing.cpp my-simulator/sensor_pointxy_bearing.h
   my-simulator/sensor_pointxy_offset.cpp my-simulator/sensor_pointxy_offset.h
   my-simulator/simutils.cpp my-simulator/simutils.h
   my-simulator/sensor_segment2d.cpp  my-simulator/sensor_segment2d.h
   my-simulator/sensor_segment2d_line.cpp  my-simulator/sensor_segment2d_line.h
   my-simulator/sensor_segment2d_pointline.cpp  my-simulator/sensor_segment2d_pointline.h
)

SET_TARGET_PROPERTIES(g2o_simulator_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}g2o_simulator_library)
target_link_libraries(g2o_simulator_library ${G2O_LIBS})

ADD_EXECUTABLE(my_simulator my-simulator/test_simulator2d_noise.cpp)
SET_TARGET_PROPERTIES(my_simulator PROPERTIES OUTPUT_NAME my_simulator${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(my_simulator g2o_simulator_library)

# my-slam build
ADD_LIBRARY(my_slam_lib ${G2O_LIB_TYPE}
  my-slam/data_association.h my-slam/data_association.cpp
  my-slam/slam_functs.h my-slam/slam_functs.cpp
)

SET_TARGET_PROPERTIES(my_slam_lib PROPERTIES OUTPUT_NAME ${LIB_PREFIX}my_slam)
target_link_libraries(my_slam_lib ${G2O_LIBS})

ADD_EXECUTABLE(my_slam my-slam/my_slam.cpp)
SET_TARGET_PROPERTIES(my_slam PROPERTIES OUTPUT_NAME my_slam${EXE_POSTFIX})
TARGET_LINK_LIBRARIES(my_slam my_slam_lib)