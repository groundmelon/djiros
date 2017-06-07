/** @file dji_sdk_node_subscriber.cpp
 *  @version 3.--------
 *  @date ----------
 *
 *  @brief
 *  ----------
 *
 *  @copyright ---- DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

void
DJISDKNode::gimbalAngleCtrlCallback(const dji_sdk::Gimbal::ConstPtr& msg)
{
  ROS_DEBUG("called gimbalAngleCtrlCallback");

  DJI::OSDK::Gimbal::AngleData angle_data;
  angle_data.duration = msg->ts;
  angle_data.mode     = msg->mode;
  angle_data.roll     = msg->roll;
  angle_data.pitch    = msg->pitch;
  angle_data.yaw      = msg->yaw;
  vehicle->gimbal->setAngle(&angle_data);
}

void
DJISDKNode::gimbalSpeedCtrlCallback(
  const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  ROS_DEBUG("called gimbalAngleCtrlCallback");

  DJI::OSDK::Gimbal::SpeedData speed_data;
  speed_data.roll  = msg->vector.y;
  speed_data.pitch = msg->vector.x;
  speed_data.yaw   = msg->vector.z;
  vehicle->gimbal->setSpeed(&speed_data);
}
