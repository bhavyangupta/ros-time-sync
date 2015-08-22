#ifndef ROS_SYNC_VID_TAG_POSE_HPP
#define ROS_SYNC_VID_TAG_POSE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <april_tag/AprilTagList.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros_time_sync/synced_frame.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <fstream>
#include <string>

#include "vid_frame_writer_t.hpp"

using namespace sensor_msgs;
using namespace message_filters;

using geometry_msgs::PoseStamped;
using april_tag::AprilTagList;
using ros_time_sync::synced_frame;

using std::string;
using std::fstream;


/**
* \class sync_publisher_t "inc/sync_vid_tag_pose.hpp"
* \brief Class to handle publishing of synched messages and message_filter 
  callbacks
*/

class sync_publisher_t {
  public:   
    sync_publisher_t(ros::NodeHandle &nh, string &video_filename, 
                     string camera_topic, string pose_topic, string tag_topic,
                     int fps, cv::Size frame_size, int disp_enable );
    sync_publisher_t(ros::NodeHandle &nh, string &video_filename, 
                     string camera_topic, string pose_topic, string tag_topic,
                     int fps, cv::Size frame_size, int disp_enable, 
                     string &text_filename);
    ~sync_publisher_t();
    void write_to_text_file(void);
    void callback_message_filter(const Image::ConstPtr &image, 
                                 const PoseStamped::ConstPtr &pose,
                                 const AprilTagList::ConstPtr &tags);

  private:
    ros::NodeHandle &sync_out_nh;
    ros::Publisher pub_sync_frame;
    synced_frame curr_data_frame;
    Subscriber<Image> sub_camera;
    Subscriber<PoseStamped> sub_pose;
    Subscriber<AprilTagList> sub_tag;
    typedef sync_policies::ApproximateTime<Image,PoseStamped,AprilTagList> sync_policies_t;
    Synchronizer<sync_policies_t> curr_sync_policy;

    string &video_filename;
    string text_filename;
    bool write_file_enable;
    fstream text_file_handle;

    const string &topic_name_camera;
    const string &topic_name_pose;
    const string &topic_name_tag;

    vid_frame_writer_t video_writer;
};


#endif //ROS_SYNC_VID_TAG_POSE_HPP