cmake_minimum_required(VERSION 3.0.2)
project(yolo_master)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
cv_bridge
sensor_msgs
image_transport
)
find_package(OpenCV 4.5.2 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES yolo_master
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(yolo_master src/yolo_master.cpp)

target_link_libraries(yolo_master 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenCV_LIBS} 
)
