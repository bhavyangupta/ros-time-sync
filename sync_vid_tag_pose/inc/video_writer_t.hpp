#ifndef VIDEO_WRITER_VIDEO_WRITER_T_HPP
#define VIDEO_WRITER_VIDEO_WRITER_T_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>           
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <iostream>

using std::string;
using std::cout;
using std::endl;

/**
* \class video_writer_t "inc/video_writer_t.hpp"
* \brief Class to wrap topic read and file write
*/

class video_writer_t {
  public:
    video_writer_t(ros::NodeHandle &nh,string &topic, string &filename,
                   int fps,cv::Size frame_size,bool disp_enable);
    ~video_writer_t();
    void callback_image_frame(const sensor_msgs::ImageConstPtr &frame);
  private:
    ros::NodeHandle writer_nh;
    ros::Subscriber sub_frame;
    string window_name;
    string &topic_name;
    string &file_name;
    int fps;
    bool display;
    cv::VideoWriter vid_out;
    cv::Size frame_size;
};

#endif //VIDEO_WRITER_VIDEO_WRITER_T_HPP
