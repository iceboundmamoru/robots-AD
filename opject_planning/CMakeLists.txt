cmake_minimum_required(VERSION 3.0.2)
project(opject_planning)

add_compile_options(-std=c++14 -Wall -Werror)

find_package(OpenCV REQUIRED)
find_package(casadi REQUIRED)
#add_subdirectory(casadi)
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
  util
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
    util
)

#SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${casadi_INCLUDE_DIRS}
)


set(opject_planning_SRC
  src/opject_planning.cpp
 
)

add_executable(opject_planning src/opject_planning.cpp  ${opject_planning_SRC})

#add_dependencies(perception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} #mpc_follower_gencfg)

target_link_libraries(opject_planning 
	${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${casadi_LIBS}

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
