cmake_minimum_required(VERSION 3.1.2)
project(basler_stereopair_driver)

set(FILESNAME BaslerStereoDriver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -pedantic -Werror -Werror=vla)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        rospy
        std_msgs
        mrs_lib
        mrs_msgs
        nav_msgs
        pcl_ros
        pcl_conversions
        tf2
        tf2_ros
        tf2_msgs
        tf2_geometry_msgs
        tf2_sensor_msgs
        apriltag_ros
        geometry_msgs
        image_geometry
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp mrs_lib geometry_msgs mrs_msgs pcl_ros nav_msgs
        DEPENDS Eigen
)

catkin_install_python(PROGRAMS scripts/pair_calibration_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )

add_library(${FILESNAME} src/${FILESNAME}.cpp)

add_dependencies(${FILESNAME} ${${FILESNAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${FILESNAME} ${catkin_LIBRARIES})
target_link_libraries(${FILESNAME} ${OpenCV_LIBRARIES})
