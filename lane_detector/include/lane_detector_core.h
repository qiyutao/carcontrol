#pragma once

#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "control_node_msgs/lane_distance.h"

#include <iostream>
#include "lane_detect_algo.h"

#include "sensor_msgs/Image.h"

class LaneDetectorCore
{
private:
  cv::Mat frame_mat; // the frame from the /image_raw topic
  cv::UMat frame_umat;

  cv_bridge::CvImagePtr cv_ptr;
  sensor_msgs::ImagePtr pub_img_msg_,pub_img_msg_mergeImage_;

  image_transport::Publisher lane_result_publisher_,mergeImage_publisher_;
  ros::Subscriber image_raw_sub_;
  ros::Publisher lane_dis_pub_;

  laneDetection detect_algo;
  // the lane distance to left right and center
  control_node_msgs::lane_distance dis_result;

  void image_raw_cb(const sensor_msgs::Image::ConstPtr &img_msg);

public:
  LaneDetectorCore(ros::NodeHandle &nh);
  ~LaneDetectorCore();

  void Spin(int freq);
};