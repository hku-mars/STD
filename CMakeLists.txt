cmake_minimum_required(VERSION 3.0.2)
project(std_detector)

## Compile as C++14
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Release")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  4")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf_conversions
)

## System dependencies are found with CMake's conventions
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED QUIET)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(place_recognition_kitti demo/place_recognition_kitti.cpp src/STDesc.cpp )
target_link_libraries(place_recognition_kitti 
        ${catkin_LIBRARIES} 
        ${PCL_LIBRARIES} 
        ${OpenCV_LIBS} 
        ${CERES_LIBRARIES})

add_executable(place_recognition_livox demo/place_recognition_livox.cpp src/STDesc.cpp )
target_link_libraries(place_recognition_livox 
        ${catkin_LIBRARIES} 
        ${PCL_LIBRARIES} 
        ${OpenCV_LIBS} 
        ${CERES_LIBRARIES})

add_executable(pgo_demo demo/pgo_demo.cpp src/STDesc.cpp )
target_link_libraries(pgo_demo 
        ${catkin_LIBRARIES} 
        ${PCL_LIBRARIES} 
        ${OpenCV_LIBS} 
        ${CERES_LIBRARIES}
        gtsam)

add_executable(online_demo demo/online_demo.cpp src/STDesc.cpp )
target_link_libraries(online_demo
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        gtsam)
