#include "vid_frame_writer_t.hpp"

/**
* \fn 
* \brief 
* \param None
* \return None
* \note 
*/
vid_frame_writer_t::vid_frame_writer_t(string output, 
                                       int FPS, 
                                       cv::Size frame_dim, 
                                       int display,
                                       string disp_name)
:output_filename(output),
 fps(FPS),
 frame_size(frame_dim),
 display_enable(display),
 window_name(disp_name)
{
  ROS_INFO_STREAM(__func__ << " "<< display<<" "<<disp_name);
  this->open_output_file();
  if(display_enable){
    ROS_INFO_STREAM("spawning_window");
    this->spawn_display_window();
  }
  return;
}

/**
* \fn 
* \brief 
* \param None
* \return None
* \note 
*/
vid_frame_writer_t::~vid_frame_writer_t(){
  if (display_enable){
    cv::destroyWindow(this->window_name);
  }
  return;
}

/**
* \fn 
* \brief 
* \param None
* \return None
* \note 
*/
void vid_frame_writer_t::open_output_file(void){
  this->vid_file_out.open(this->output_filename,CV_FOURCC('M','J','P','G'),
                          this->fps, frame_size,true);
  if (!vid_file_out.isOpened()){
    ROS_ERROR("Video output file cannot be opened");
    exit(-1);
  }
  return;
}

/**
* \fn 
* \brief 
* \param None
* \return None
* \note 
*/
void vid_frame_writer_t::spawn_display_window(void){
  // ROS_INFO_STREAM(__func__);
  cv::namedWindow(this->window_name);
  return;
}

/**
* \fn 
* \brief Wrapper around display_frame and write_frame. Displays frame if enabled, 
         else just writes it to file.
* \param None
* \return None
* \note 
*/
void vid_frame_writer_t::process_frame(const sensor_msgs::ImageConstPtr &frame_msg){
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(frame_msg,sensor_msgs::image_encodings::BGR8);
  last_frame = cv_ptr->image;
  if(this->display_enable){
    this->display_frame();
  }
  this->write_frame();
  return; 
}


/**
* \fn 
* \brief 
* \param None
* \return None
* \note 
*/
void vid_frame_writer_t::display_frame(void){
  // ROS_INFO_STREAM(__func__);
  cv::imshow(this->window_name,last_frame);
  cv::waitKey(1000/this->fps);
  return;
}

/**
* \fn 
* \brief 
* \param None
* \return None
* \note 
*/
void vid_frame_writer_t::write_frame(){
  vid_file_out.write(last_frame);
  return;
}


