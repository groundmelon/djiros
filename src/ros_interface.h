#ifndef __ROS_INTERFACE_H
#define __ROS_INTERFACE_H

#include <ros/ros.h>
#include "SDK.h"
#include <sensor_msgs/Joy.h>

#define TIME_DIFF_CHECK 0.010
#define TIME_DIFF_ALERT 0.020
#define ALIGN_BUFFER_SIZE 100

enum ALIGN_STATE_t {ALIGN_UNINITED=0, ALIGN_RUNNING=1, ALIGN_FINISHED=3};
enum CTRL_STATE_t {CTRL_STOP=0, CTRL_ACQUIRING=1, CTRL_ACQUIRED=2, CTRL_RUNNING=3, CTRL_SIGNAL_LOST=4};

struct Align_data_t
{
	ros::Time time;
	uint32_t  tick;
};

void interface_init(ros::NodeHandle& nh);
void interface_flush_time();
void ros_process_sdk_std_msg(const sdk_std_msg_t& recv_sdk_std_msgs, uint16_t msg_flags);
void interface_control_callback(const sensor_msgs::Joy& msg);
void interface_control_timer(const ros::TimerEvent& e);
void stop_control();
void ctrl_acquire_ack_success();
void ctrl_release_ack_success();
void api_acquire_control();
void api_release_control();

#endif