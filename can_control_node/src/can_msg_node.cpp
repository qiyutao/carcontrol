#include "can_msg_node.h"

CanMsgNode::CanMsgNode(ros::NodeHandle &nh)
{
    // nh.param<std::string>("can_msg_pub_topic", can_msg_pub_topic_, "/can_msg_received");
    // ROS_INFO("can_msg_pub_topic_: %s", can_msg_pub_topic_.c_str());

    steer_msg_sub_ = nh.subscribe("/vehicle/steering_cmd", 50, &CanMsgNode::SteerCmdCb, this);
    brake_msg_sub_ = nh.subscribe("/vehicle/brake_cmd", 50, &CanMsgNode::BrakeCmdCb, this);
    throttle_msg_sub_ = nh.subscribe("/vehicle/throttle_cmd", 50, &CanMsgNode::ThrottleCmdCb, this);
    closest_waypoint_sub_ = nh.subscribe("/closest_waypoint", 50, &CanMsgNode::closest_waypointCb, this);

    eps_msg_pub_ = nh.advertise<can_node_msg::steel_report>("eps_msg", 10);
    break_msg_pub_ = nh.advertise<can_node_msg::break_report>("break_msg", 10);
    throttle_msg_pub_ = nh.advertise<can_node_msg::throttle_report>("throttle_msg", 10);

    steering_cmd_ = 0.0;
    brake_cmd_ = 0.0;
    throttle_cmd_ = 0.0;

    //最大转角角速度  degree/s
    max_steer_speed = 300;
    current_steer_angle_ = 0.0;
    freq_counter = 0;

    //init the cmd value
    steel_manual_ = 0.0;
    break_manual_ = 0.0;
    throttle_manual_ = 0.0;

    // init the param
    vci_device_type_ = 4;
    vci_device_ind_ = 0;
    can_port_idx_ = 0;
    wait_time = 20;

    freq_counter = 0;

    // set the config structure
    config.AccCode = 0x80000008;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;     //接收所有帧
    config.Timing0 = 0x00; /*波特率500 Kbps  0x03  0x1C*/
    config.Timing1 = 0x1C;
    config.Mode = 0; //正常模式

    if (PrepareCan())
    {
        std::cout << "Prepare can successfully!!" << std::endl;
    }
}

CanMsgNode::~CanMsgNode()
{
}

int CanMsgNode::PrepareCan()
{
    VCI_OpenDevice(vci_device_type_, vci_device_ind_, 0);
    if (VCI_InitCAN(vci_device_type_, vci_device_ind_, can_port_idx_, &config) != 1)
    {
        ROS_ERROR(">>Init CAN %d error\n", can_port_idx_);
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return 0;
    }

    if (VCI_InitCAN(vci_device_type_, vci_device_ind_, 1, &config) != 1)
    {
        ROS_ERROR(">>Init CAN 2 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return 0;
    }

    if (VCI_StartCAN(vci_device_type_, vci_device_ind_, can_port_idx_) != 1)
    {
        ROS_ERROR(">>Start CAN %d error\n", can_port_idx_);
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return 0;
    }

    return 1;
}

can_node_msg::can CanMsgNode::can_obj2msg(const VCI_CAN_OBJ &can_obj)
{
    can_node_msg::can can_msg;
    //set Header with ros time rather than can_obj time, in case there are no timestamp
    can_msg.header.frame_id = "/can_msg";
    can_msg.header.stamp = ros::Time::now();

    //set data
    can_msg.id = can_obj.ID;
    can_msg.timeflag = can_obj.TimeFlag;
    can_msg.sendtype = can_obj.SendType;
    can_msg.remoteflag = can_obj.RemoteFlag;
    can_msg.externflag = can_obj.ExternFlag;
    can_msg.datalen = can_obj.DataLen;
    for (int i = 0; i < 8; i++)
    {
        can_msg.data[i] = can_obj.Data[i];
    }
    return can_msg;
}

void CanMsgNode::SteerCmdCb(const control_node_msgs::SteeringCmd::ConstPtr &msg)
{
    steering_cmd_ = msg->steering_cmd;
}
void CanMsgNode::BrakeCmdCb(const control_node_msgs::BrakeCmd::ConstPtr &msg)
{
    brake_cmd_ = msg->brake_cmd;
}
void CanMsgNode::ThrottleCmdCb(const control_node_msgs::ThrottleCmd::ConstPtr &msg)
{
    throttle_cmd_ = msg->throttle_cmd;
}

void CanMsgNode::SendSteerMsg(double steerling_value, unsigned int w_mode)
{
    steerling_frame.SendType = 0;
    steerling_frame.RemoteFlag = 0;
    steerling_frame.ExternFlag = 0;
    steerling_frame.DataLen = 8;
    for (int j = 0; j < 8; j++)
    {
        //将数据位初始化为0
        steerling_frame.Data[j] = 0;
    }
    steerling_frame.ID = STEERLING_TX;

    // steering data
    // steering angle data
    double fix_steer_center = steerling_value - 8.0;

    // this is the final check for the steering angle
    if (fix_steer_center < -430.0)
    {
        fix_steer_center = -430.0;
    }
    if (fix_steer_center > 430.0)
    {
        fix_steer_center = 430.0;
    }
    unsigned int steer_signal = (fix_steer_center + 870.0) * 10.0;
    unsigned int steer_high = steer_signal >> 8;
    unsigned int steer_low = steer_signal & 0x00FF;
    steerling_frame.Data[0] = steer_high;
    steerling_frame.Data[1] = steer_low;

    // work mode: 0待机 1自动驾驶 2保留 4手动  5手动介入恢复  6清故障
    // adas state, 1ADAS状态正常 0ADAS状态不正常
    // steer state 1转向指令正常  0转向指令异常
    unsigned int work_mode;

    if (w_mode == 1)
    {
        work_mode = 1;
        if ((steel_manual_ == 5) | (steel_manual_ == 6) | (steel_manual_ == 7))
        {
            // 如果上层下发指令为自动驾驶，但是此时EPS仍然为手动介入恢复或警告模式,则清理
            work_mode = 6;
        }
    }

    unsigned int adas_state = 1 << 4;
    unsigned int steer_state = 1 << 5;
    steerling_frame.Data[2] = (steer_state | adas_state | work_mode);

    //  最大角速度  degree/s, with a P controller
    double steer_angle_cte = std::fabs(fix_steer_center - current_steer_angle_);
    double tmp_steering_speed = steer_angle_cte * 10.0;

    // 最大角速度是540度/s 最小角速度是100度/s
    tmp_steering_speed = std::min(std::max(tmp_steering_speed, 100.0), 540.0);

    steerling_frame.Data[4] = tmp_steering_speed / 10.0;
    // 设置辅助力矩数值
    steerling_frame.Data[6] = 0x80;
    //通讯计数
    // send_frame[0].Data[6] = counter & 0x0F;
    //check sum
    steerling_frame.Data[7] = steerling_frame.Data[0] + steerling_frame.Data[1] + steerling_frame.Data[2] +
                              steerling_frame.Data[3] + steerling_frame.Data[4] + steerling_frame.Data[5] +
                              steerling_frame.Data[6];

    VCI_Transmit(vci_device_type_, vci_device_ind_, can_port_idx_, &steerling_frame, 1);
}

void CanMsgNode::SendThrottleMsg(double throttle_value, unsigned int man_control, unsigned int adas_state)
{
    throttle_frame.SendType = 0;
    throttle_frame.RemoteFlag = 0;
    throttle_frame.ExternFlag = 0;
    throttle_frame.DataLen = 8;
    for (int j = 0; j < 8; j++)
    {
        //将数据位初始化为0
        throttle_frame.Data[j] = 0;
    }
    throttle_frame.ID = THROTTLE_TX;

    //throttle data test vehicle
    unsigned int throttle_signal = 5000.0 + throttle_value * 100.0;

    //product vehicle
    // unsigned int throttle_signal = throttle_value * 100.0;
    // throttle high
    throttle_frame.Data[0] = throttle_signal >> 8;
    // throttle low
    throttle_frame.Data[1] = throttle_signal & 0x00FF;

    // 是否允许人工驾驶： 0：允许人工踩油门， 1：不允许人工踩油门
    throttle_frame.Data[2] = man_control;

    //Adas 状态： 0：人工驾驶状态  1：自动驾驶状态  2：Adas故障
    throttle_frame.Data[3] = adas_state;

    //check sum
    throttle_frame.Data[7] = throttle_frame.Data[0] + throttle_frame.Data[1] + throttle_frame.Data[2] +
                             throttle_frame.Data[3] + throttle_frame.Data[4] + throttle_frame.Data[5] +
                             throttle_frame.Data[6];

    VCI_Transmit(vci_device_type_, vci_device_ind_, can_port_idx_, &throttle_frame, 1);
}

void CanMsgNode::SendBreakMsg(double break_value)
{
    break_frame.SendType = 0;
    break_frame.RemoteFlag = 0;
    break_frame.ExternFlag = 0;
    break_frame.DataLen = 8;
    for (int j = 0; j < 8; j++)
    {
        //将数据位初始化为0
        break_frame.Data[j] = 0;
    }
    break_frame.ID = BREAK_TX;
    // break data
    int break_signal = 1000.0 * break_value;
    // break high
    break_frame.Data[0] = break_signal >> 8;
    // break low
    break_frame.Data[1] = break_signal & 0x00FF;
    // decel request active (0, 1)
    break_frame.Data[2] = 1;

    break_frame.Data[7] = break_frame.Data[0] + break_frame.Data[1] + break_frame.Data[2] +
                          break_frame.Data[3] + break_frame.Data[4] + break_frame.Data[5] +
                          break_frame.Data[6];
    VCI_Transmit(vci_device_type_, vci_device_ind_, can_port_idx_, &break_frame, 1);
}

void CanMsgNode::closest_waypointCb(const std_msgs::Int32ConstPtr &msg)
{
    int way_point_index = msg->data;
    std::cout << "way_point_index:" << way_point_index << std::endl;
    // stop at the end point
    if (way_point_index > END_WAYPOINT)
    {
        break_applied_ = true;
    }
}

void CanMsgNode::SendCanMsg(double steerling_value, double break_value, double throttle_value, bool auto_enable)
{
}

void CanMsgNode::ReceiveCanMsg()
{
    // before receive the can msg, check the msg number in the receive buffer, we get all
    // buffered msgs at one time
    unsigned int buffered_frame_num = VCI_GetReceiveNum(vci_device_type_, vci_device_ind_, can_port_idx_);
    ROS_INFO("buffered frame %d", buffered_frame_num);

    if (buffered_frame_num > 100)
    {
        // if the buffered msg num is more than the structure size
        buffered_frame_num = 100;
    }
    if (buffered_frame_num > 0)
    {
        unsigned int receive_len = 0;
        receive_len = VCI_Receive(vci_device_type_, vci_device_ind_, can_port_idx_, can_obj_, buffered_frame_num, wait_time);
        ROS_INFO("received len is %d", receive_len);
        for (int j = 0; j < receive_len; j++)
        {
            if (can_obj_[j].ID == STEERLING_RX)
            {
                // The steerling msg received from the car
                ProcessSteelMsg(can_obj_[j]);
            }
            if (can_obj_[j].ID == BREAK_RX)
            {
                // The break msg received from the car
                ProcessBreakMsg(can_obj_[j]);
            }
            if (can_obj_[j].ID == THROTTLE_RX)
            {
                // The throttle msg received from the car
                ProcessThrottleMsg(can_obj_[j]);
            }
        }
    }
}

void CanMsgNode::ProcessBreakMsg(VCI_CAN_OBJ &can_obj)
{
    // 我们可以从刹车的can帧中获得当前车速（km/h）
    unsigned int current_speed_signal = (can_obj.Data[1] << 8) | (can_obj.Data[2]);
    unsigned int current_speed = current_speed_signal * 0.007813;
    unsigned int aeb_active = can_obj.Data[0] & 0x80;
    unsigned int aeb_not_active = can_obj.Data[0] & 0x40;
    unsigned int acc_not_avaiable = can_obj.Data[0] & 0x20;
    unsigned int aeb_break_active = can_obj.Data[0] & 0x10;
    unsigned int break_applied = can_obj.Data[0] & 0x8;

    // 判断是否有人踩下刹车，人踩下刹车，退出无人驾驶模式
    if (break_applied == 8)
    {
        break_applied_ = true;
    }
    else
    {
        break_applied_ = false;
    }

    can_node_msg::break_report report;

    report.header.frame_id = "/break_msg";
    report.header.stamp = ros::Time::now();
    report.speed = current_speed;
    report.aeb_active = aeb_active;
    report.aeb_not_active = aeb_not_active;
    report.acc_not_avaiable = acc_not_avaiable;
    report.aeb_break_active = aeb_break_active;
    report.break_applied = break_applied;

    break_msg_pub_.publish(report);
}

void CanMsgNode::ProcessThrottleMsg(VCI_CAN_OBJ &can_obj)
{
    //反馈的油门系数
    unsigned int r_throttle_signal = (can_obj.Data[0] << 8) | (can_obj.Data[1]);
    double r_throttle_real = r_throttle_signal;
    // 是否人工介入：0人未踩   1人踩下
    throttle_manual_ = can_obj.Data[2];

    // 油门控制器状态 0:人工， 1：自动
    unsigned int drive_state = can_obj.Data[3];

    can_node_msg::throttle_report report;
    report.header.frame_id = "/throttle_msg";
    report.header.stamp = ros::Time::now();
    report.throttle = r_throttle_real;
    report.throttle_manual = throttle_manual_;
    report.drive_state = drive_state;

    throttle_msg_pub_.publish(report);
}

void CanMsgNode::ProcessSteelMsg(VCI_CAN_OBJ &can_obj)
{
    // the eps state:
    // work mode: 0待机 1自动驾驶 2保留 4手动  5手动介入恢复  6警告模式 7错误模式
    steel_manual_ = can_obj.Data[0];
    //eps返回的转角角速度
    unsigned int steel_speed = can_obj.Data[1] * 10;
    // eps返回的方向盘转角
    unsigned int r_angle_signal = (can_obj.Data[2] << 8) | (can_obj.Data[3]);
    double r_angle_real = r_angle_signal * 0.1 - 870;
    //eps返回的力矩值
    double torgue_value = can_obj.Data[4] * 0.055 - 7;

    can_node_msg::steel_report report;
    report.header.frame_id = "/steel_msg";
    report.header.stamp = ros::Time::now();

    report.steel_manual = steel_manual_;
    report.steel_speed = steel_speed;
    report.steel_angle = r_angle_real;
    current_steer_angle_ = r_angle_real;
    report.torgue_value = torgue_value;

    eps_msg_pub_.publish(report);
}

void CanMsgNode::EndSignalHandler(int sig)
{

    ROS_INFO("program finished!!!");
    ros::shutdown();
}

void CanMsgNode::Spin(int freq)
{
    ros::Rate r(freq);
    while (ros::ok())
    {
        ReceiveCanMsg();
        ros::spinOnce();
        // process();
        r.sleep();
    }
}

void CanMsgNode::process()
{
    // the steerling angle is (左)430---(右)-430
    // if ((steel_manual_ == 5) | (steel_manual_ == 6))
    // {
    //     work_mode = 6;
    // }
    if (break_applied_)
    {
        // 工作模式4: 人工驾驶模式
        SendSteerMsg(0.0, 4);
        SendBreakMsg(0.0);
        // man_control 0：允许人类踩油门  drive state 0: 人工驾驶模式
        SendThrottleMsg(0.0, 0, 0);
    }
    else
    {
        // SendSteerMsg(steering_cmd_, 1);
        SendBreakMsg(brake_cmd_);
        // SendThrottleMsg(throttle_cmd_, 0, 1);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_msg_receiver");

    ros::NodeHandle nh;

    CanMsgNode can_receiver(nh);
    can_receiver.Spin(50);
    return 0;
}