cmake_minimum_required(VERSION 2.8.3)
project(e100_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)

 add_message_files(
   FILES
   Num.msg
 )

 add_service_files(
   FILES
   AddTwoInts.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_tele src/joy_tele.cpp)
target_link_libraries(joy_tele ${catkin_LIBRARIES})
add_dependencies(joy_tele ria_teleop_generate_messages_cpp)

install(
  DIRECTORY launch config msg srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS joy_tele
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}
)