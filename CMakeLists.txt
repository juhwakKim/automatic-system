cmake_minimum_required(VERSION 2.8.3)
project(automatic-system)

# Set c++11 cmake flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


# Find rquired packeges
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    roscpp
    rospy
    std_msgs
    actionlib
    darknet_ros_msgs
    image_transport
    tf
)



catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    cv_bridge
    roscpp
    actionlib
    rospy
    std_msgs
    darknet_ros_msgs
    image_transport
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} DIRECTORY include/)

add_executable(compress src/compressed.cpp)
add_executable(obj_cor src/object_cordinate.cpp)

target_link_libraries(compress ${catkin_LIBRARIES}  ${OpenCV_LIBS})
target_link_libraries(obj_cor ${catkin_LIBRARIES} ${OpenCV_LIBS})

