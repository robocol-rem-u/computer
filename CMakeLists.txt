cmake_minimum_required(VERSION 2.8.3)
project(computer)
find_package(catkin REQUIRED 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs
  message_generation
)



#add_service_files(
#  FILES
#  enable.srv
#)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  roscpp
  rospy
  std_msgs
  geometry_msgs
)


# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
