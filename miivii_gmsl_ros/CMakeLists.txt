cmake_minimum_required(VERSION 2.8.3)
project(miivii_gmsl_ros)

set (CMAKE_CXX_STANDARD 11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             image_transport
             roscpp
             std_msgs
             std_srvs
             sensor_msgs
             cv_bridge
             camera_info_manager)

find_package(OpenCV REQUIRED )
find_package(CUDA REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system thread)
## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
set(linux_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_path(MIIVIICAMSDK_INCLUDES MvGmslCamera.h
          /opt/miivii/include)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
  ${MIIVIICAMSDK_INCLUDES}
  ${linux_INCLUDE_DIR}
)

link_directories( ${catkin_LIB_DIRS} /opt/miivii/lib)

## Declare a cpp executable
set(NODE_NAME ${PROJECT_NAME}_node)
add_executable(${PROJECT_NAME}_node nodes/miivii_gmsl_ros_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
  ${catkin_LIB_DIRS} mvgmslcam
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
