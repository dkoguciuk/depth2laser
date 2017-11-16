cmake_minimum_required(VERSION 2.8.3)
project(depth2laser)

# Fix for compilation error on Ubuntu 16.04
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORCE_INLINES")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    tf
    tf_conversions
    pcl_conversions
    cv_bridge
    nodelet
)

#####################################################################
########################## EXTERNAL LIBRARIES #######################
#####################################################################


# PCL
find_package(PCL 1.7.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find Boost
find_package(Boost 1.54 REQUIRED system chrono)
if (Boost_FOUND)
    message("-- Boost ${Boost_VERSION} was found")
    include_directories(${Boost_INCLUDE_DIRS})
else(Boost_FOUND)
    message(FATAL_ERROR "Boost 1.54 was not found")
endif(Boost_FOUND)

# Find OpenCV
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message("-- OpenCV ${OpenCV_VERSION} was found")
    include_directories(${OpenCV_INCLUDE_DIRS})
else(OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV was not found")
endif(OpenCV_FOUND)

# CUDA
find_package(CUDA REQUIRED)
if (CUDA_FOUND)
    message("-- CUDA ${CUDA_VERSION} was found")
    include(FindCUDA)
    include_directories(${CUDA_INCLUDE_DIRS})
else(CUDA_FOUND)
    message("-- CUDA was not found")
    message(FATAL_ERROR "Not all CUDA libraries have been found")
endif(CUDA_FOUND)

#####################################################################
########################### CATKIN MAGIC ############################
#####################################################################

## Catkin
catkin_package(
    INCLUDE_DIRS include
#    LIBRARIES depth2laser
    CATKIN_DEPENDS roscpp sensor_msgs tf tf_conversions pcl_conversions
#    DEPENDS system_lib
)

#####################################################################
########################### VTK MACRO DEF ###########################
#####################################################################

set(DEPTH2LASER_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
macro(REMOVE_VTK_DEFINITIONS MODULE_NAME)
     get_directory_property(_dir_defs DIRECTORY ${DEPTH2LASER_SOURCE_DIR} COMPILE_DEFINITIONS)
     set(_vtk_definitions)
     foreach(_item ${_dir_defs})
         if(_item MATCHES "vtk*")
             list(APPEND _vtk_definitions -D${_item})
         endif()
     endforeach()
     remove_definitions(${_vtk_definitions})
endmacro(REMOVE_VTK_DEFINITIONS)

# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

#####################################################################
############################ CUDA LIBRARY ###########################
#####################################################################

#definitions
add_definitions(-DPROJECT_SOURCE_DIR="\\"${PROJECT_SOURCE_DIR}\\"")

# Pass options to NVCC (for Jetson TK1)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -gencode arch=compute_30,code=sm_30 --ptxas-options=-v --maxrregcount=32)

# Add cuda library for files using cuda methods
remove_vtk_definitions(depth2laser_cuda)
cuda_add_library(depth2laser_cuda src/kernels/gpu_utils.cu)

#####################################################################
############################# C++11 #################################
#####################################################################

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#####################################################################
############################## NODELET ##############################
#####################################################################

# depth2laser nodelet
add_library(depth2laser_nodelet include/gpu_utils.h
    src/transform_matrix.cpp include/transform_matrix.h
    src/depth2laser_nodelet.cpp include/depth2laser_nodelet.h)
target_link_libraries(depth2laser_nodelet ${catkin_LIBRARIES} ${PCL_LIBRARIES} depth2laser_cuda -lboost_system -lboost_chrono)

#####################################################################
############################### NODES ###############################
#####################################################################

add_executable(depth2laser_node src/depth2laser_node.cpp)
target_link_libraries(depth2laser_node depth2laser_cuda ${catkin_LIBRARIES} ${PCL_LIBRARIES} -lboost_system -lboost_chrono)

# z_calibration
add_executable(z_calib src/main_z_calib.cpp)
target_link_libraries(z_calib ${catkin_LIBRARIES} ${PCL_LIBRARIES})
