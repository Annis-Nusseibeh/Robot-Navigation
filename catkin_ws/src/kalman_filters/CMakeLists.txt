cmake_minimum_required(VERSION 3.0)
project(kalman_filters)

link_directories(/usr/local/include)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
    sensor_msgs
    geometry_msgs
    cmake_modules
    message_generation
)

add_message_files(
  FILES
  ahrs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  MESSAGE(STATUS "Not Found")
else()
  MESSAGE(STATUS "Found")
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(include ${catkin_INCLUDE_DIRS} $(EIGEN3_INCLUDE_DIRS))

catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime)

add_executable(attitude_kf_node src/Attitude_KF.cpp)
add_dependencies(attitude_kf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(attitude_kf_node ${catkin_LIBRARIES})

add_executable(bno055_node src/BNO055_Subscriber.cpp)
target_link_libraries(bno055_node ${catkin_LIBRARIES})