cmake_minimum_required(VERSION 3.2)
project(dmp_control)

add_definitions(-DLINUX)

set(CMAKE_CXX_STANDARD 11) # Tell CMake to compile with C++11

# Moc generated files are located in the current dir so we need to tell CMake to look for them.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  #OpenCV REQUIRED
  #PCL REQUIRED
  roscpp
  roslib
  message_generation
  std_msgs
  geometry_msgs
  dmp_lib
  io_lib
  math_lib
  autharl_core
  lwr_robot
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS opencv pcl eigen
)

set(SRC_LIST

    src/Controller.cpp

)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  /usr/include/eigen3
)

# *************************************************
# *************************************************

add_executable(dmp_control
  src/main.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(dmp_control
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(dmp_control
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  dmp-lib
  io-lib
  math-lib
  autharl-robot
  autharl-utils
  lwr-robot
)
