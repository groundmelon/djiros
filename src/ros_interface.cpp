/******** RC Map *********
*
*  +8000 <--->  0  <---> -8000
*   API  <---> ATT <--->  POS
*
*        CH3 +10000                     CH1 +10000
*               ^                              ^
*               |                              |                   / -10000
*    CH2        |                   CH0        |                  /
*  -10000 <-----------> +10000    -10000 <-----------> +10000    H
*               |                              |                  \
*               |                              |                   \ -4545
*               V                              V
*            -10000                         -10000
*
********** Frames **********
*
*       N(orth)                    U(p)                  F(orward)
*      /                           |                    /
*     /                            |    F(orward)      /
*    /______ E(ast)                |  /               /______ R(ight)
*    |                             | /                |
*    |                      ______ |/                 |
*    |D(own)               L(eft)                     |D(own)
*
*
****************************************/

#include "ros_interface.h"

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <list>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>

#define _TICK2ROSTIME(tick) (ros::Duration((double)tick / 600.0))

using namespace std;
using namespace Eigen;

ros::Time base_time;
ALIGN_STATE_t align_state;
extern double g_gravity;

ros::Publisher pub_imu;
ros::Publisher pub_velo;
ros::Publisher pub_gps;
ros::Publisher pub_mag;
ros::Publisher pub_rc;
ros::Publisher pub_gimbal;
ros::Publisher pub_time_ref;

list<Align_data_t> alignArray;
Matrix3d ros_R_fc;

api_ctrl_without_sensor_data_t ctrl_cmd;
ros::Time last_ctrl_time;
bool ctrl_updated;
CTRL_STATE_t ctrl_state;
ros::Time ctrl_acquire_start_time;
bool ctrl_acquire_acked;

// ---------------------------------
// implementation is in dji_sdk_node
void api_acquire_control();
void api_release_control();
// ---------------------------------

void interface_init(ros::NodeHandle& nh)
{
	ctrl_updated = false;
	ctrl_state = CTRL_STOP;
	last_ctrl_time = ros::Time::now();
	align_state = ALIGN_UNINITED;
	alignArray.clear();

	ros_R_fc << 1,0,0,0,-1,0,0,0,-1;

	pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 10);
	pub_velo = nh.advertise<geometry_msgs::Vector3Stamped>("velo", 10);
	pub_gps = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);
	pub_mag = nh.advertise<sensor_msgs::MagneticField>("mag", 10);
	pub_rc = nh.advertise<sensor_msgs::Joy>("rc", 10);
	pub_gimbal = nh.advertise<geometry_msgs::Vector3Stamped>("gimbal", 10);
	pub_time_ref = nh.advertise<sensor_msgs::TimeReference>("tick", 10);
}

void ros_process_sdk_std_msg(const sdk_std_msg_t& recv_sdk_std_msgs,  uint16_t msg_flags)
{
	ros::Time now_time = ros::Time::now();

	// only use 100HZ imu bag (0x3f) to align
	if (align_state == ALIGN_UNINITED && ((msg_flags & 0x3F)==0x3F) )
	{
		base_time = now_time - _TICK2ROSTIME(recv_sdk_std_msgs.time_stamp);
		align_state = ALIGN_RUNNING;
		ROS_ERROR("[info] start imu align");
		return;
	}

	if (align_state == ALIGN_RUNNING && ((msg_flags & 0x3F)==0x3F) )
	{
		if (alignArray.size() < ALIGN_BUFFER_SIZE)
		{
			Align_data_t data = {
				.time = now_time,
				.tick = recv_sdk_std_msgs.time_stamp
			};
			alignArray.push_back(data);
		}
		else
		{
			assert(alignArray.size()==ALIGN_BUFFER_SIZE);
			bool all_sample_pass_test = true;
			for(auto it=alignArray.begin(); it!=alignArray.end(); ++it)
			{
				double dt = (it->time - (base_time + _TICK2ROSTIME(it->tick))).toSec();
				if (std::fabs(dt) > TIME_DIFF_CHECK)
				{
					all_sample_pass_test = false;
					alignArray.erase(alignArray.begin(), std::next(it));
					break;
				}
			}

			if (all_sample_pass_test)
			{
				ROS_ERROR("***** IMU ALIGNED *****");
				align_state = ALIGN_FINISHED;
			}
			else
			{
				Align_data_t data = {
					.time = now_time,
					.tick = recv_sdk_std_msgs.time_stamp
				};
				alignArray.push_back(data);
				base_time = now_time - _TICK2ROSTIME(alignArray.front().tick);
			}
		}
		return;
	}

	if (align_state == ALIGN_FINISHED)
	{
		ros::Time tick_time = base_time + _TICK2ROSTIME(recv_sdk_std_msgs.time_stamp);
		double dt = (now_time - tick_time).toSec();
// printf("recv:%d\ntick%f\n",
// 	recv_sdk_std_msgs.time_stamp,
// 	tick_time.toSec()
// 	);
		if (std::fabs(dt) > TIME_DIFF_ALERT)
		{
			ROS_WARN("[WARN] SysTime - TickTime = %.0f ms", dt*1000);
		}

		Quaterniond q;
		q.w() = recv_sdk_std_msgs.q.q0;
		q.x() = recv_sdk_std_msgs.q.q1;
		q.y() = recv_sdk_std_msgs.q.q2;
		q.z() = recv_sdk_std_msgs.q.q3;

		Matrix3d gRb = q.toRotationMatrix();

#if 0
		// accel in std_msg is acceleration related to ground
		Vector3d a_g;
		a_g(0) = recv_sdk_std_msgs.a.x;
		a_g(1) = recv_sdk_std_msgs.a.y;
		a_g(2) = recv_sdk_std_msgs.a.z;
		Vector3d a_b = /* gRb.transpose() */ a_g * 9.8;
#else
		// accel in std_msg is the raw acceleration output from IMU
		Vector3d a_b;
		a_b(0) = recv_sdk_std_msgs.a.x;
		a_b(1) = recv_sdk_std_msgs.a.y;
		a_b(2) = recv_sdk_std_msgs.a.z;
		a_b = a_b * g_gravity;
#endif

		Vector3d v_g;
		v_g(0) = recv_sdk_std_msgs.v.x;
		v_g(1) = recv_sdk_std_msgs.v.y;
		v_g(2) = recv_sdk_std_msgs.v.z;

		sensor_msgs::Imu imu_msg;
		{
			imu_msg.header.stamp = tick_time;
			imu_msg.header.frame_id = std::string("FLU");
			Quaterniond q_ros(Matrix3d(ros_R_fc * gRb * ros_R_fc.transpose()));
			imu_msg.orientation.x = q_ros.x();
			imu_msg.orientation.y = q_ros.y();
			imu_msg.orientation.z = q_ros.z();
			imu_msg.orientation.w = q_ros.w();
			imu_msg.angular_velocity.x =  recv_sdk_std_msgs.w.x;
			imu_msg.angular_velocity.y = -recv_sdk_std_msgs.w.y;
			imu_msg.angular_velocity.z = -recv_sdk_std_msgs.w.z;
			imu_msg.linear_acceleration.x =  a_b(0);
			imu_msg.linear_acceleration.y = -a_b(1);
			imu_msg.linear_acceleration.z = -a_b(2);
		}

		geometry_msgs::Vector3Stamped velo_msg;
		{
			velo_msg.header.stamp = tick_time;
			velo_msg.header.frame_id = std::string("NED");
			velo_msg.vector.x = recv_sdk_std_msgs.v.x;
			velo_msg.vector.y = recv_sdk_std_msgs.v.y;
			velo_msg.vector.z = recv_sdk_std_msgs.v.z;
		}

		sensor_msgs::NavSatFix gps_msg;
		{
			gps_msg.header.stamp = tick_time;
			gps_msg.header.frame_id = std::string("NED");
			gps_msg.latitude = recv_sdk_std_msgs.pos.lati;
			gps_msg.longitude = recv_sdk_std_msgs.pos.longti;
			// gps_msg.altitude = recv_sdk_std_msgs.pos.alti;
			gps_msg.altitude = recv_sdk_std_msgs.pos.height;
			gps_msg.status.status = recv_sdk_std_msgs.pos.health_flag;
		}

		sensor_msgs::MagneticField mag_msg;
		{
			mag_msg.header.stamp = tick_time;
			mag_msg.header.frame_id = std::string("NED");
			mag_msg.magnetic_field.x = recv_sdk_std_msgs.mag.x;
			mag_msg.magnetic_field.y = recv_sdk_std_msgs.mag.y;
			mag_msg.magnetic_field.z = recv_sdk_std_msgs.mag.z;
		}

		sensor_msgs::Joy rc_msg;
		{
			rc_msg.header.stamp = tick_time;
			rc_msg.header.frame_id = std::string("rc");
			rc_msg.axes.push_back((double)recv_sdk_std_msgs.rc.roll/10000.0);
			rc_msg.axes.push_back((double)recv_sdk_std_msgs.rc.pitch/10000.0);
			rc_msg.axes.push_back((double)recv_sdk_std_msgs.rc.yaw/10000.0);
			rc_msg.axes.push_back((double)recv_sdk_std_msgs.rc.throttle/10000.0);
			rc_msg.axes.push_back((double)recv_sdk_std_msgs.rc.mode/10000.0);
			rc_msg.axes.push_back((double)recv_sdk_std_msgs.rc.gear/10000.0);

			rc_msg.buttons.push_back(recv_sdk_std_msgs.status);
			rc_msg.buttons.push_back(recv_sdk_std_msgs.battery_remaining_capacity);
			rc_msg.buttons.push_back(recv_sdk_std_msgs.ctrl_device);
		}

		geometry_msgs::Vector3Stamped gmb_msg;
		{
			gmb_msg.header.stamp = tick_time;
			gmb_msg.header.frame_id = std::string("gimbal");
			gmb_msg.vector.x = recv_sdk_std_msgs.gimbal.x;
			gmb_msg.vector.y = recv_sdk_std_msgs.gimbal.y;
			gmb_msg.vector.z = recv_sdk_std_msgs.gimbal.z;
		}

		sensor_msgs::TimeReference tmr_msg;
		{
			tmr_msg.header.stamp = tick_time;
			tmr_msg.time_ref = ros::Time((double)recv_sdk_std_msgs.time_stamp);
		}

		if ((msg_flags & ENABLE_MSG_Q) && (msg_flags & ENABLE_MSG_A) && (msg_flags & ENABLE_MSG_W))
		{
			pub_imu.publish(imu_msg);
		}

		if ((msg_flags & ENABLE_MSG_V))
		{
			pub_velo.publish(velo_msg);
		}

		if ((msg_flags & ENABLE_MSG_POS))
		{
			pub_gps.publish(gps_msg);
		}

		if ((msg_flags & ENABLE_MSG_MAG))
		{
			pub_mag.publish(mag_msg);
		}

		if ((msg_flags & ENABLE_MSG_RC))
		{
			pub_rc.publish(rc_msg);
		}

		if ((msg_flags & ENABLE_MSG_GIMBAL))
		{
			pub_gimbal.publish(gmb_msg);
		}

		if ((msg_flags & ENABLE_MSG_TIME))
		{
			pub_time_ref.publish(tmr_msg);
		}


		// judge mode select channel to acquire control
		if (recv_sdk_std_msgs.rc.mode > 5000)
		{
			if (ctrl_state==CTRL_STOP)
			{
				ctrl_state = CTRL_ACQUIRING;
				ctrl_acquire_start_time = ros::Time::now();
				ctrl_acquire_acked = false;

				api_acquire_control();
			}
		}
		else
		{
			ctrl_state = CTRL_STOP;
		}
	}
}

void interface_control_callback(const sensor_msgs::Joy& msg)
{
	if (msg.header.frame_id.compare("FRD")==0)
	{
		last_ctrl_time = ros::Time::now();
		ctrl_cmd.roll_or_x = msg.axes[0];
		ctrl_cmd.pitch_or_y = msg.axes[1];
		ctrl_cmd.thr_z = msg.axes[2];
		ctrl_cmd.yaw = msg.axes[3];
		if (msg.axes[4] > 0)
		{
			ctrl_cmd.ctrl_flag = 0b00100010; // 0b00100000, mode 13
		}
		else
		{
			ctrl_cmd.ctrl_flag = 0b00000010; // 0b00000000, mode 1
		}
		ctrl_updated = true;
	}
	else
	{
		ROS_ERROR("In N1Ctrl: input joy_msg.frame_id should be FRD!!!");
	}
}

void interface_control_timer(const ros::TimerEvent& e)
{
	if (ctrl_state == CTRL_STOP || ctrl_state == CTRL_SIGNAL_LOST)
	{
		return;
	}
	else
	{
		if ((ros::Time::now() - last_ctrl_time).toSec() > 0.200)
		{
			ROS_ERROR("[ERR] Controll signal lost!!");
			ctrl_state = CTRL_SIGNAL_LOST;
			api_release_control();
			return;
		}
	}

	if (ctrl_state == CTRL_ACQUIRING)
	{
		if ((ros::Time::now()-ctrl_acquire_start_time).toSec() > 1.0)
		{
			ctrl_state = CTRL_STOP;
			ROS_ERROR("[ERR] Acquire control failed.");
		}
		else
		{
			// wait for timeout
		}
	}
	else if (ctrl_state == CTRL_ACQUIRED)
	{
		ROS_ERROR("****** Acquire control success! ******");
		ctrl_state = CTRL_RUNNING;
	}
	else if (ctrl_state == CTRL_RUNNING)
	{
		if (ctrl_updated)
		{
			App_Send_Data(0, 0, MY_CTRL_CMD_SET, API_CTRL_REQUEST, (uint8_t*)&ctrl_cmd, sizeof(ctrl_cmd), NULL, 0, 0);
		}
	}
	else
	{
		assert(false && "Invalid logic branch in timer callback.");
	}

}

void stop_control()
{
	ctrl_state = CTRL_STOP;
}

void ctrl_acquire_ack_success()
{
	ctrl_state = CTRL_ACQUIRED;
	last_ctrl_time = ros::Time::now();
	ROS_INFO("Ncore acquire acked.");
}

void ctrl_release_ack_success()
{
	ROS_ERROR("------ Release control success! ------");
}
