cmake_minimum_required(VERSION 3.20)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(ROS_ORB_SLAM3)

rosbuild_init()
message("----------------------初始化完成！-----------------------------------")

set(ROS_BUILD_TYPE Release)
# set(ROS_BUILD_TYPE Debug)

message("Build type222: " ${ROS_BUILD_TYPE}) # Release
message("---------------------------------------------------------")

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
message("---------------------------------------------------------")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

# 要保证整个工程的 opencv 版本一致，包括 dbow，源码以及 ros 相关的
# 3 4 都可以正常运行
find_package(OpenCV 4 REQUIRED)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV 4 not found.")
endif()
message("OPENCV VERSION: ${OpenCV_VERSION}") # 4.2.0
message("OpenCV include directories: ${OpenCV_INCLUDE_DIRS}") # /usr/include/opencv4
message("OpenCV libraries: ${OpenCV_LIBS}") # many many
message("---------------------------------------------------------")

find_package(Eigen3 REQUIRED)
message("Eigen3 version: ${Eigen3_VERSION}")
message("---------------------------------------------------------")
find_package(Pangolin REQUIRED)
message("Pangolin version: ${Pangolin_VERSION}")
message("---------------------------------------------------------")
find_package(Sophus REQUIRED)
message("Sophus version: ${Sophus_VERSION}")
message("---------------------------------------------------------")

include_directories(
   ${PROJECT_SOURCE_DIR}
   ${PROJECT_SOURCE_DIR}/../../../
   ${PROJECT_SOURCE_DIR}/../../../include
   ${PROJECT_SOURCE_DIR}/../../../include/CameraModels
   # ${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
   ${Pangolin_INCLUDE_DIRS}
   ${Sophus_INCLUDE_DIRS}
)

set(LIBS
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${Pangolin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
   ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
   ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
   -lboost_system
)

message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}") # PROJECT SOURCE DIR: /home/dsg/orbslam3_ws/src/ORB_SLAM3_comments/Examples/ROS/ORB_SLAM3
message("---------------------------------------------------------")

# Node for monocular camera
rosbuild_add_executable(Mono src/ros_mono.cc) # 单目
target_link_libraries(Mono ${LIBS})

# Node for stereo camera
rosbuild_add_executable(Stereo src/ros_stereo.cc) # 双目
target_link_libraries(Stereo ${LIBS})

# Node for RGB-D camera
rosbuild_add_executable(RGBD src/ros_rgbd.cc) # RGB-D
target_link_libraries(RGBD ${LIBS})

# Node for monocular-inertial camera
rosbuild_add_executable(Mono_Inertial src/ros_mono_inertial.cc) # 单目 + IMU
target_link_libraries(Mono_Inertial ${LIBS})

# Node for stereo-inertial camera
rosbuild_add_executable(Stereo_Inertial src/ros_stereo_inertial.cc) # 双目 + IMU
target_link_libraries(Stereo_Inertial ${LIBS})

# Node for rgbd-inertial camera
rosbuild_add_executable(RGBD_Inertial src/ros_rgbd_inertial.cc) # RGB-D + IMU
target_link_libraries(RGBD_Inertial ${LIBS})
