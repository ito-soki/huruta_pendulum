cmake_minimum_required(VERSION 2.8.3)
project(huruta_pendulum_controller)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -mtune=native -mavx2 -mfma")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime 
)

include_directories(
 include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


include_directories("/usr/local/include")

add_library(pendulum_controller src/pendulum_controller.cpp)
target_link_libraries(pendulum_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})




