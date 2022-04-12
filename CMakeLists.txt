cmake_minimum_required(VERSION 3.1.2)
project(basler_stereopair_driver)

set(FILESNAME BaslerStereoDriver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -pedantic -Werror -Werror=vla)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        mrs_lib
        nav_msgs
        apriltag_ros
        geometry_msgs
        image_geometry
        message_generation
        )

find_package(OpenCV REQUIRED)

add_message_files(
        FILES
        ImageMultiview.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS message_runtime roscpp mrs_lib geometry_msgs
)

catkin_install_python(PROGRAMS scripts/pair_calibration_node.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

include_directories(include
        ${catkin_INCLUDE_DIRS}
        )

add_library(${FILESNAME} src/${FILESNAME}.cpp)

add_dependencies(${FILESNAME} ${${FILESNAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${FILESNAME} ${catkin_LIBRARIES})
