#include "lane_detector_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_msg_receiver");

    ros::NodeHandle nh;

    LaneDetectorCore lane_detector_core(nh);
    lane_detector_core.Spin(20);
    return 0;
}