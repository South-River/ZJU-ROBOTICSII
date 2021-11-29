cmake_minimum_required(VERSION 2.8.3)
project(course_agv_nav)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  Plan.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES course_agv_nav
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories()

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library( course_agv_nav
  src/map.cpp
)

add_executable(map_server src/map.cpp)
target_link_libraries(map_server ${catkin_LIBRARIES})
