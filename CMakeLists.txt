project(lidar)
cmake_minimum_required(VERSION 3.6)
add_definitions(-std=c++0x)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#GSL
find_package(GSL REQUIRED)
include_directories(${GSL_INCLUDE_DIRS} ${GSLCBLAS_INCLUDE_DIRS})
set(GSLLIB ${LIBS} ${GSL_LIBRARIES} ${GSLCBLAS_LIBRARIES})

set(SOURCES src/Kalman.cpp include/Kalman.h)

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ./include
)

catkin_package(
)

# editted by sam in 2018-1-17
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS} ${GSLLIB})

add_executable(kalman ${SOURCES})
target_link_libraries(kalman
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBS}
        ${GSLLIB}
)
