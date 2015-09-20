/** \mainpage 
* \section intro_sec Introduction
* ROS node to synchronise video frames, april tag data and pose generated 
* from Hector SLAM
* \section use Usage
  rosrun ros_time_sync sync_vid_tag_pose_node _video_name:=out 
  _camera_topic:=/usb_cam/image_raw _width:=480 _height:=270 _pose_topic:=/slam_out_pose 
  _tag_topic:=/april_tags _enable_text_log:=0 _text_log_name:=filename.txt


* \section description Parameter Desciption
  [MANDATORY] _video_name:="video -> Name of the video file 

  [MANDATORY] _camera_topic:=""
  [MANDATORY] _width:=
  [MANDATORY] _height:=  
  [MANDATORY] _fps:=
  [MANDATORY] _display:=
  [MANDATORY] _pose_topic:=""
  [MANDATORY] _tag_topic:=""


  [MANDATORY] _enable-text-log:=1 -> Flag to enable text file writing. If 
  disabled, only the topic is published and no file is written. If enabled,it is
  mandatory to provide the next parameter.

  [OPTIONAL] _text_log_name:="filename" -> Name of output text file.

* \warning  Make sure that no other file exists with the same name or else the 
  code exits with error.

* \section topic Topics Used
* \subsection subscribed Subscribed
* \subsection published  Published
   synced_tag_and_pose of type ros_time_sync::synced_frame
* \section output_file Text file format:

Entries are stored row-wise in the following format:

[secs:nsecs],robot.position.x,robot.position.y,robot.position.z,
robot.orientation.x,robot.orientation.y,robot.orientation.z,robot.orientation.w,
number_of_tags,tag1_id,tag1.position.x,tag1.position.y,tag1.position.z,tag1.orientation.roll,
tag1.orientaion.pitch,tag1.orientation.yaw....

* \warning  This code is based a highly modified version of the april_tag package
* \author Bhavya Narain Gupta
* \date 
*/

#include "sync_vid_tag_pose.hpp"
#include <iostream>

using std::cout;
using std::endl;

void prompt_usage(void) {
   ROS_ERROR("Usage: rosrun ros_time_sync sync_vid_tag_pose_node \
    _video-name:=out.mkv _camera-topic:= /usb_cam/image_raw _pose-topic:=/slam_out_pose - \
  _tag-topic:=/april_tags _enable-text-log:=1 _text-log-name:=filename.txt");
  return;
}

int main(int argc, char ** argv){
  cout << argc << endl;
  if(argc < 3){ 
    ROS_ERROR("Insufficient Parameters");
    prompt_usage();    
    exit(-1);
  }

  string video_filename = "";
  string text_filename = "";
  string topic_camera_str = "";
  string topic_pose_str = "";
  string topic_tag_str = "";
  int text_logging = 0;
  int frame_width = 0;
  int frame_height = 0;
  int disp_enable = 0;
  int fps = 0;
  bool disp_flag = false;

  ros::init(argc, argv,"sync_vid_tag_pose_node");
  ros::NodeHandle nh("~");
  nh.getParam("video_name",video_filename);
  nh.getParam("camera_topic",topic_camera_str);
  nh.getParam("pose_topic",topic_pose_str);
  nh.getParam("tag_topic",topic_tag_str);
  nh.getParam("enable_text_log",text_logging);
  nh.getParam("text_log_name",text_filename);
  nh.getParam("width",frame_width);
  nh.getParam("height",frame_height);
  nh.getParam("fps",fps);
  nh.getParam("display",disp_enable);

  video_filename = video_filename + ".mkv";
  text_filename = text_filename + ".txt";


  cv::Size frame_size(frame_width, frame_height);

  if(text_logging){
    if(text_filename == ""){
      ROS_ERROR ("Output text file name missing");
      prompt_usage();
      exit(-1);
    }
    else{    
      ROS_INFO_STREAM("Sync with text file output: "<< text_filename);
      sync_publisher_t sync_with_file(nh,video_filename,topic_camera_str,
                                      topic_pose_str,topic_tag_str,fps,
                                      frame_size,disp_enable,text_filename);
      ros::spin();
    }
  }

  else if (!text_logging){
    ROS_INFO_STREAM("Sync without text file output");
    ROS_INFO_STREAM(video_filename<<" "<<topic_camera_str<<" "<<topic_pose_str <<" "<<topic_tag_str);
    sync_publisher_t sync_without_file(nh,video_filename,topic_camera_str,
                                       topic_pose_str,topic_tag_str,fps,
                                       frame_size,disp_enable);
  
    ros::spin();
  }
  return 0;

  // TODO: handle default topic names here.
}