#ifndef __ROS_INTERFACE_H
#define __ROS_INTERFACE_H

#include <ros/ros.h>
#include "SDK.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>

#define TIME_DIFF_CHECK 0.010
#define TIME_DIFF_ALERT 0.020
#define TIME_DIFF_STOP_SYNC 0.002
#define SYNC_STEP 0.001
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
void interface_gimbal_control_callback(const geometry_msgs::PoseStampedConstPtr& msg);
void interface_control_timer(const ros::TimerEvent& e);
void stop_control();
void ctrl_acquire_ack_success();
void ctrl_release_ack_success();
void api_acquire_control();
void api_release_control();
bool activate_is_successful();
void activate_ack_handle(unsigned short ack);

class SDKSyncronizer
{
public:
	int count;
	ros::Time src_time;
	ros::Time des_time;
	ros::Duration step_dt;
	bool finished;

	SDKSyncronizer()
	{
		count = 0;
		src_time = ros::Time(0);
		finished = true;
	};

	SDKSyncronizer(ros::Time& _tm, double dt)
	{
		count = 0;
		src_time = _tm;
		step_dt = ros::Duration(SYNC_STEP * dt / std::fabs(dt));
		finished = false;
		ROS_WARN("[djiros]<sync> start base[%.3f] dt=%.3f", src_time.toSec(), dt);
	};

	ros::Time update(const ros::Time& base_time, const double dt)
	{
		assert(!finished);

		if (std::fabs(dt) < TIME_DIFF_STOP_SYNC)
		{
			stop();
			return base_time;
		}
		else
		{
			des_time = base_time + step_dt;
			count++;
			// ROS_INFO("[djiros]<sync> base[%.3f]->[%.3f] dt=%.3f steps=%d",
			// 	src_time.toSec(), 
			// 	des_time.toSec(), 
			// 	dt,
			// 	count);
			return des_time;
		}
	};
	
	void stop()
	{
		ROS_INFO("[djiros]<sync> end base[%.3f]--[%.3f]->[%.3f] steps=%d",
			src_time.toSec(), 
			(des_time-src_time).toSec(),
			des_time.toSec(), 
			count
		);
		finished = true;
	};
};

#endif