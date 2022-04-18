cmake_minimum_required(VERSION 2.4.6)
project(tf_test)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        nav_msgs
        roslib
        tf
        message_generation

        #Eigen3
        )
include_directories(
        #include
        ${catkin_INCLUDE_DIRS}
        #"/usr/include/eigen3"
)
add_executable(pubtf src/pubtf.cpp)
target_link_libraries(pubtf ${catkin_LIBRARIES})