set(ROS_BUILD_TYPE Debug)
cmake_minimum_required(VERSION 2.8.3)
project(ros_time_sync)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        sensor_msgs
                                        message_filters
                                        geometry_msgs
                                        cv_bridge
                                        image_transport
                                        sensor_msgs
                                        )
message(INFO "******************* catkin: " ${catkin_LIBRARIES})
message(INFO "******************* roscpp: " ${roscpp_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS date_time system thread)
message(INFO "${Boost_LIBRARIES}")


include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(inc/)

add_executable(sync_vid_tag_pose_node 
               src/main.cpp 
               src/sync_vid_tag_pose.cpp 
               src/video_writer_t.cpp)
add_dependencies(sync_vid_tag_pose_node april_tag_generate_messages_cpp)

target_link_libraries(sync_vid_tag_pose_node ${catkin_LIBRARIES} 
                                             ${OpenCV_LIBRARIES})