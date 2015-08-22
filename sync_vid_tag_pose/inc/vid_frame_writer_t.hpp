#ifndef SYNC_VID_TAG_POSE_VID_FRAME_WRITER_T
#define SYNC_VID_TAG_POSE_VID_FRAME_WRITER_T

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <string>

using std::string;

/**
* \class vid_frame_writer_t "inc/vid_frame_writer_t.hpp"
* \brief Class to handle saving / display of synced video frames
*/

class vid_frame_writer_t {  
  public:
    vid_frame_writer_t(string output, int FPS,cv::Size frame_dim,int display,
                       string disp_name = "output");
    ~vid_frame_writer_t(void);
    void open_output_file(void);
    void spawn_display_window(void);
    void process_frame(const sensor_msgs::ImageConstPtr &frame_msg);
    void display_frame(void);
    void write_frame(void);

  private:
    cv::Mat last_frame;
    cv::Size frame_size;
    cv::VideoWriter vid_file_out;
    int fps;
    string output_filename;
    string window_name;
    int display_enable;
};
#endif //SYNC_VID_TAG_POSE_VID_FRAME_WRITER_T