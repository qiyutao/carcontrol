#include "lane_detector_core.h"

LaneDetectorCore::LaneDetectorCore(ros::NodeHandle &nh)
{
    image_raw_sub_ = nh.subscribe("/image_raw", 20, &LaneDetectorCore::image_raw_cb, this);

    image_transport::ImageTransport it(nh);
    lane_result_publisher_ = it.advertise("/detect_lane_image", 10);
    mergeImage_publisher_= it.advertise("/detect_merge_image", 10);
    lane_dis_pub_ = nh.advertise<control_node_msgs::lane_distance>("/lane_distance", 20);
}

LaneDetectorCore::~LaneDetectorCore()
{
}

void LaneDetectorCore::Spin(int freq)
{
    ros::Rate r(freq);
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}

void LaneDetectorCore::image_raw_cb(const sensor_msgs::Image::ConstPtr &img_msg)
{
    ros::Time start_time = ros::Time::now();
    try
    {
        //将sensor_msgs::Image数据类型转换为cv::Mat类型（即opencv的IplImage格式）
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    frame_mat = cv_ptr->image;

    frame_umat = frame_mat.getUMat(cv::ACCESS_RW);
    //input the image to the algorithm
    detect_algo.setInputImage(frame_umat);
    detect_algo.laneDetctAlgo();

    dis_result.header.frame_id = "lane distance";
    dis_result.header.stamp = ros::Time::now();
    vector<float> dis_tmp = detect_algo.getLaneCenterDist();
    dis_result.dis_to_center = dis_tmp[0];
    dis_result.dis_to_left = dis_tmp[1];
    dis_result.dis_to_right = dis_tmp[2];
    lane_dis_pub_.publish(dis_result);

    cv::Mat publish_mat = detect_algo.getFinalResult().getMat(cv::ACCESS_RW);
     cv::Mat publish_mergeImage = detect_algo.getMergeImage().getMat(cv::ACCESS_READ);

    ros::Time end_time = ros::Time::now();
    // std::cout<<"time for process 1 frame:"<<end_time-start_time<<std::endl;
    if (!publish_mat.empty())
    {
        pub_img_msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", publish_mat).toImageMsg();
        lane_result_publisher_.publish(pub_img_msg_);
        //cv::waitKey(1);
    }
    if (!publish_mergeImage.empty())
    {
        pub_img_msg_mergeImage_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", publish_mergeImage).toImageMsg();
        mergeImage_publisher_.publish(pub_img_msg_mergeImage_);
        //cv::waitKey(1);
    }
}
