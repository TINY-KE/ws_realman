cmake_minimum_required(VERSION 2.8.3)
project(view_planning_3d_real)

# 各类库
set(OpenCV_DIR "/home/robotlab/thirdparty/opencv-3.4.10/build")



SET(CMAKE_BUILD_TYPE Debug)  #zhjd

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(cv_bridge_DIR "/home/robotlab/thirdparty/for_dspslam/vision_opencv/cv_bridge/build")

find_package(catkin REQUIRED COMPONENTS
        actionlib
        control_msgs
        diagnostic_msgs
        geometry_msgs
        # manipulation_msgs   #used for ubuntu18
        moveit_msgs
        moveit_ros_perception
        moveit_ros_planning_interface
        roscpp
        rospy
        sensor_msgs
        shape_msgs
        std_msgs
        std_srvs
        tf
        trajectory_msgs
        visualization_msgs
        rviz_visual_tools
        moveit_visual_tools
        gazebo_msgs
        tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(gazebo REQUIRED)


# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(gpmp2 REQUIRED)
include_directories(${gpmp2_INCLUDE_DIR})

include_directories(
        ${GAZEBO_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

# 包含 OpenCV 头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(  cv_bridge )

catkin_package(
        # INCLUDE_DIRS include
        # LIBRARIES connect_gazebo
        # CATKIN_DEPENDS roscpp std_msgs  gazebo_msgs
        # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}
)



find_package(Boost REQUIRED COMPONENTS filesystem system)


set(SOURCE_LZW_QUADRIC_FILES
        src/plane/Plane.cpp
        src/Converter.cc
        src/Map.cpp
        src/MapObject.cpp
        src/TeleopMobile.cpp
        src/Viewer.cpp
        #      src_object/core/ConstrainPlane.cpp
        #      src_object/core/Ellipsoid.cpp
)


# 二、GTSAM-QUADRIC 相关  
set(SOURCE_GTSAM_QUADRIC_FILES
        ./src_gtsam_quadrics/Utilities.cpp
        ./src_gtsam_quadrics/ConstrainedDualQuadric.cpp
        ./src_gtsam_quadrics/AlignedBox2.cpp
        ./src_gtsam_quadrics/AlignedBox3.cpp
        ./src_gtsam_quadrics/BoundingBoxFactor.cpp
        ./src_gtsam_quadrics/QuadricCamera.cpp
        ./src_gtsam_quadrics/DualConic.cpp
)



add_library(CONVENIENCE_LIB_NAME SHARED
        ${SOURCE_LZW_QUADRIC_FILES}
        ${SOURCE_GTSAM_QUADRIC_FILES}
)

target_link_libraries(CONVENIENCE_LIB_NAME
        ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${GTSAM_LIBRARIES} gpmp2 ${OpenCV_LIBS}
)



# 系统版本
add_executable(real-ASLAM src_ros/real-ASLAM.cpp
        #         ${SOURCE_LZW_QUADRIC_FILES}
        #         ${SOURCE_GTSAM_QUADRIC_FILES}
)
target_link_libraries(real-ASLAM
        ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${GTSAM_LIBRARIES} gpmp2 ${OpenCV_LIBS}  CONVENIENCE_LIB_NAME )

