cmake_minimum_required(VERSION 2.8.3)
project(navigation_kalman_filter)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs)

catkin_package(
        INCLUDE_DIRS include
        #LIBRARIES phidgets_imu
        CATKIN_DEPENDS roscpp sensor_msgs std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_library(navigation_kalman_filter src/Navigation_Kalman_Filter.cpp)

add_executable(navigation_kalman_filter_node src/Navigation_Kalman_Filter.cpp)

#add_dependencies(navigation_kalman_filter ${catkin_EXPORTED_TARGETS})
add_dependencies(navigation_kalman_filter_node ${catkin_EXPORTED_TARGETS})

#target_link_libraries(navigation_kalman_filter ${catkin_LIBRARIES})
target_link_libraries(navigation_kalman_filter_node ${catkin_LIBRARIES})

#roslaunch_add_file_check(launch)
