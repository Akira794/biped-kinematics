cmake_minimum_required(VERSION 2.8.3)
project(pcpc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories( ${catkin_INCLUDE_DIRS})

add_executable(ggp_test src/main.cpp)
target_link_libraries(ggp_test ${catkin_LIBRARIES})
