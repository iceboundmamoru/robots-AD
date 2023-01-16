cmake_minimum_required(VERSION 3.0.2)
project(perception)

add_compile_options(-std=c++14 -Wall -Werror)

find_package(OpenCV REQUIRED)
#find_package(OpenCV2 REQUIRED)

find_package(
  catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rosunit
  std_msgs
  tf2
  tf2_ros
  sensor_msgs
  cv_bridge
)



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    std_msgs
    tf2
    tf2_ros
    sensor_msgs
    cv_bridge
)

#SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
 
)


set(PERCEPTION_SRC
  src/perception.cpp
 
)

add_executable(perception src/perception.cpp  ${PERCEPTION_SRC})

#add_dependencies(perception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} #mpc_follower_gencfg)

target_link_libraries(perception 
	${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#install(
#  TARGETS perception
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#install(
#  DIRECTORY
#    launch
#    config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
