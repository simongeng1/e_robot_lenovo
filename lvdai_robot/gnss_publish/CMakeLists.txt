cmake_minimum_required(VERSION 2.8.3)
project(gnss_publish)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  serial
  nmea_msgs
  
  tf
)

## Generate messages in the 'msg' folder
add_message_files(FILES
  gnss_gphcd.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gnss_publish
  CATKIN_DEPENDS roscpp rospy std_msgs serial  nmea_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
  ${catkin_INCLUDE_DIRS}
)




add_executable(gnss_data
  src/gnss_data.cpp
)
add_dependencies(gnss_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gnss_data
  ${catkin_LIBRARIES}
)


add_executable(gnss_pub_sentence
  src/gnss_pub_sentence.cpp
)
add_dependencies(gnss_pub_sentence ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gnss_pub_sentence
  ${catkin_LIBRARIES}
)

add_executable(gnss_sub
  src/gnss_sub.cpp
)
add_dependencies(gnss_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gnss_sub
  ${catkin_LIBRARIES}
)