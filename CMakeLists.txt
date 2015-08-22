cmake_minimum_required(VERSION 2.8.3)
project(ros_time_sync)
set(ROS_BUILD_TYPE Debug)
message(STATUS ${CMAKE_PREFIX_PATH})

set(april_tag_DIR "")
message(ONE)
find_package(catkin REQUIRED)
find_package(april_tag)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        message_generation
                                        std_msgs
                                        geometry_msgs)


include_directories(${catkin_INCLUDE_DIRS} ${april_tag_INCLUDE_DIRS})
message(APRIL_TAG PATH: )
message(status ${april_tag_INCLUDE_DIRS})

# Message generation
message(asdbfsa)
add_message_files( 
  FILES   
  synced_frame.msg
)
  
message(three)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  april_tag
)

message(four)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs april_tag)

add_subdirectory(sync_vid_tag_pose/)
