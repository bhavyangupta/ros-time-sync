#include "sync_vid_tag_pose.hpp"
#include <iostream>

/**
* \fn sync_publisher_t::sync_publisher_t(ros::NodeHandle &nh, string &video_name)
* \brief Constructor - invoked when text file output is DISABLED
* \param nh Reference to invoking node handle
* \return None
*/
sync_publisher_t::sync_publisher_t(ros::NodeHandle &nh, 
                                   string &video_name, 
                                   string camera_topic,
                                   string pose_topic,
                                   string tag_topic,
                                   int fps,
                                   cv::Size frame_size,
                                   int disp_enable)
: sync_out_nh(nh),
  video_filename(video_name),
  topic_name_camera(camera_topic),
  topic_name_pose(pose_topic),
  topic_name_tag(tag_topic),
  sub_camera(nh,camera_topic,30),
  sub_pose(nh,pose_topic,30),
  sub_tag(nh,tag_topic,30),
  curr_sync_policy(sync_policies_t(30),sub_camera,sub_pose,sub_tag),
  video_writer(video_name, fps, frame_size, disp_enable)
{ 
  ROS_INFO_STREAM("Hey there");
  curr_sync_policy.registerCallback(boost::bind(&sync_publisher_t::callback_message_filter,this,_1,_2,_3));
  ROS_INFO_STREAM(__func__<<"after callback");
  pub_sync_frame = sync_out_nh.advertise<synced_frame>("synced_tag_and_pose",10);

  return;
}

/**
* \fn sync_publisher_t::sync_publisher_t(ros::NodeHandle &nh, string &video_name
                                         string &text_name)
* \brief Constructor - invoked when text file output is ENABLED
* \param nh Reference to invoking node ha1ndle
* \note This returns an error and exits process if the text filename provided 
        already exists. This is done to prevent accidental overwrites of 
        previous logs.
* \return None
*/
sync_publisher_t::sync_publisher_t(ros::NodeHandle &nh, 
                                   string &video_name,
                                   string camera_topic,
                                   string pose_topic,
                                   string tag_topic,
                                   int fps,
                                   cv::Size frame_size, 
                                   int disp_enable,
                                   string &text_name)
: sync_out_nh(nh),
  video_filename(video_name),
  topic_name_camera(camera_topic),
  topic_name_pose(pose_topic),
  topic_name_tag(tag_topic),
  sub_camera(nh,camera_topic,10),
  sub_pose(nh,pose_topic,10),
  sub_tag(nh,tag_topic,10),
  text_filename(text_name),
  curr_sync_policy(sync_policies_t(10),sub_camera,sub_pose,sub_tag),
  video_writer(video_name, fps, frame_size, disp_enable)
{
  ROS_INFO_STREAM(__func__);
  curr_sync_policy.registerCallback(boost::bind(&sync_publisher_t::callback_message_filter,this,_1,_2,_3));
  const char * text_filename_char = text_filename.c_str();
  pub_sync_frame = sync_out_nh.advertise<synced_frame>("synced_tag_and_pose",10);
  // TODO: Open file here and assign handle -> error check [DONE]
  write_file_enable = true;
  // Open the file to see if it exists:
  text_file_handle.open(text_filename_char,std::ios::in);
  if(text_file_handle) { // File exists - close it and exit on error.
    text_file_handle.close();
    ROS_ERROR("Output text file cannot be opened. Make sure provided text filename is unique to avoid overwriting an existing file.");
    exit(-1);
  }
  //  if the file didn't exist - open a new one for writing
  else { 
    text_file_handle.open(text_filename_char,std::ios::out);
  }

  return;
}


/**
* \fn sync_publisher_t::~sync_publisher_t()
* \brief Destructor

  Checks for open text file and closes it if needed.
* \param None
* \return None
*/
sync_publisher_t::~sync_publisher_t(){
  ROS_INFO_STREAM(__func__);
  // close the file here if writing enabled
  if(write_file_enable){
    if(text_file_handle.is_open()){
      text_file_handle.close();
    }
  }
  return;
}

/**
* \fn sync_publisher_t::write_to_text_file() 
* \brief Writes current data_frames to csv file if text file output is enabled

Entries are stored row-wise in the following format:

[secs:nsecs],robot.position.x,robot.position.y,robot.position.z,
robot.orientation.x,robot.orientation.y,robot.orientation.z,robot.orientation.w,
number_of_tags,tag1_id,tag1.position.x,tag1.position.y,tag1.position.z,tag1.orientation.roll,
tag1.orientaion.pitch,tag1.orientation.yaw....

* \param None
* \return None
* \note Robot orientation is a quaternion while tag orientation is a set of 
        Euler angles
*/
void sync_publisher_t::write_to_text_file() {
  text_file_handle<<"["<<curr_data_frame.header.stamp.sec<<":"
                  <<curr_data_frame.header.stamp.nsec<<"]"<<","
                  <<curr_data_frame.slam_pose.position.x<<","
                  <<curr_data_frame.slam_pose.position.y<<","
                  <<curr_data_frame.slam_pose.position.z<<","
                  <<curr_data_frame.slam_pose.orientation.x<<","
                  <<curr_data_frame.slam_pose.orientation.y<<","
                  <<curr_data_frame.slam_pose.orientation.z<<","
                  <<curr_data_frame.slam_pose.orientation.w<<",";
  
  int number_of_tags = curr_data_frame.tag_list.n_tags;
  text_file_handle<<number_of_tags;
  
  if (number_of_tags == 0){ 
    // If there are no tags, move to a new line
    text_file_handle<<"\n";
  }
  else{
    text_file_handle<<",";
    for (int i = 0;i<number_of_tags;i++){
      text_file_handle<<curr_data_frame.tag_list.april_tags[i].id<<",";
      text_file_handle<<curr_data_frame.tag_list.april_tags[i].x<<",";
      text_file_handle<<curr_data_frame.tag_list.april_tags[i].y<<",";
      text_file_handle<<curr_data_frame.tag_list.april_tags[i].z<<",";
      text_file_handle<<curr_data_frame.tag_list.april_tags[i].roll<<",";
      text_file_handle<<curr_data_frame.tag_list.april_tags[i].pitch<<",";
      text_file_handle<<curr_data_frame.tag_list.april_tags[i].yaw<<",";
    }
    text_file_handle<<"\n";
  }

  return;
}

/**
* \fn sync_publisher_t::callback_message_filter (const Image::ConstPtr &image, 
                                                const PoseStamped::ConstPtr &pose,
                                                const AprilTagList:: &tags)
* \brief Callback for approximate time message filter

  This function is called whenever the approximate time algorithm can synch
  messages with time stamp as close as possible. The function updates the 
  internal "curr_frame" data member with the passed pose and april tag list. It 
  also writes this data to a csv file with the current header timestamp
* \param &image Constant reference to image frame
* \param &pose Constant reference to slam generated pose
* \param &tag_list Constant reference to list of april tags.
* \return None
*/
void sync_publisher_t::callback_message_filter(const Image::ConstPtr &image, 
                                               const PoseStamped::ConstPtr &lidar_pose,
                                               const AprilTagList::ConstPtr &tags){
  ROS_INFO_STREAM(__func__);
  curr_data_frame.header.stamp = ros::Time::now(); // new time stamp
  curr_data_frame.tag_list.n_tags = tags->n_tags;
  curr_data_frame.tag_list.april_tags = tags->april_tags;
  curr_data_frame.slam_pose.position.x = lidar_pose->pose.position.x;
  curr_data_frame.slam_pose.position.y = lidar_pose->pose.position.y;
  curr_data_frame.slam_pose.position.z = lidar_pose->pose.position.z;
  curr_data_frame.slam_pose.orientation.x = lidar_pose->pose.orientation.x;
  curr_data_frame.slam_pose.orientation.y = lidar_pose->pose.orientation.y;
  curr_data_frame.slam_pose.orientation.z = lidar_pose->pose.orientation.z;
  curr_data_frame.slam_pose.orientation.w = lidar_pose->pose.orientation.w;

  if(write_file_enable){
    write_to_text_file();
  }
  // call ros::NodeHandle::publish here
  pub_sync_frame.publish(curr_data_frame);
  video_writer.process_frame(image);
  return;
}