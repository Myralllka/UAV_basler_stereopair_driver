cmake_minimum_required(VERSION 3.1.2)
project(basler_stereopair_driver)

set(FILESNAME BaslerStereoDriver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -pedantic -Werror -Werror=vla)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mrs_lib
  nav_msgs
  apriltag_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp mrs_lib 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(${FILESNAME} src/${FILESNAME}.cpp)

add_dependencies(${FILESNAME} ${${FILESNAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${FILESNAME} ${catkin_LIBRARIES})
