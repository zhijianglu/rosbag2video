cmake_minimum_required(VERSION 2.8.3)
project(rosbag_tools)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")


find_package(OpenCV 3.4 REQUIRED)
find_package(Eigen3 3 REQUIRED)

set(LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${catkin_LIBRARIES}
        )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imagePublisher
  CATKIN_DEPENDS cv_bridge image_transport  roscpp sensor_msgs std_msgs tf

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(rosbag2video src/rosbag2video.cpp)
target_link_libraries(rosbag2video
        ${LIBS}
        #        ${catkin_LIBRARIES}
        )
