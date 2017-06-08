/** @file dji_sdk.h
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  This file lists all functionalities as a part of
 *	the messages, services and actions in ROS.
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

#ifndef SDK_LIBRARY_H
#define SDK_LIBRARY_H

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

//! SDK library
#include <djiosdk/dji_vehicle.hpp>

//! ROS standard msgs
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

////msgs
#include <djiros/FlightControl.h>
#include <djiros/FlightControlAdvanced.h>
#include <djiros/FlightControlAngularRateVertPos.h>
#include <djiros/FlightControlAttiVertPos.h>
#include <djiros/FlightControlPosYaw.h>
#include <djiros/FlightControlVelYawRate.h>
#include <djiros/Gimbal.h>
#include <djiros/MobileData.h>

////srvs
//! service headers
#include <djiros/Activation.h>
#include <djiros/CameraAction.h>
#include <djiros/DroneArmControl.h>
#include <djiros/DroneTaskControl.h>
#include <djiros/MFIOConfig.h>
#include <djiros/MFIOSetValue.h>
#include <djiros/SDKControlAuthority.h>
#include <djiros/SendMobileData.h>

//! mission service headers
// missionManager
#include <djiros/MissionStatus.h>
// waypoint
#include <djiros/MissionWpAction.h>
#include <djiros/MissionWpGetInfo.h>
#include <djiros/MissionWpGetSpeed.h>
#include <djiros/MissionWpSetSpeed.h>
#include <djiros/MissionWpUpload.h>
// hotpoint
#include <djiros/MissionHpAction.h>
#include <djiros/MissionHpGetInfo.h>
#include <djiros/MissionHpResetYaw.h>
#include <djiros/MissionHpUpdateRadius.h>
#include <djiros/MissionHpUpdateYawRate.h>
#include <djiros/MissionHpUpload.h>
// hardsync
#include <djiros/SetHardSync.h>

namespace dji_sdk=djiros;

#endif
