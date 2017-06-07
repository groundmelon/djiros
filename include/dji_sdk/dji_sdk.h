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
#include <dji_sdk/FlightControl.h>
#include <dji_sdk/FlightControlAdvanced.h>
#include <dji_sdk/FlightControlAngularRateVertPos.h>
#include <dji_sdk/FlightControlAttiVertPos.h>
#include <dji_sdk/FlightControlPosYaw.h>
#include <dji_sdk/FlightControlVelYawRate.h>
#include <dji_sdk/Gimbal.h>
#include <dji_sdk/MobileData.h>

////srvs
//! service headers
#include <dji_sdk/Activation.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/MFIOConfig.h>
#include <dji_sdk/MFIOSetValue.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/SendMobileData.h>

//! mission service headers
// missionManager
#include <dji_sdk/MissionStatus.h>
// waypoint
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionWpGetInfo.h>
#include <dji_sdk/MissionWpGetSpeed.h>
#include <dji_sdk/MissionWpSetSpeed.h>
#include <dji_sdk/MissionWpUpload.h>
// hotpoint
#include <dji_sdk/MissionHpAction.h>
#include <dji_sdk/MissionHpGetInfo.h>
#include <dji_sdk/MissionHpResetYaw.h>
#include <dji_sdk/MissionHpUpdateRadius.h>
#include <dji_sdk/MissionHpUpdateYawRate.h>
#include <dji_sdk/MissionHpUpload.h>
// hardsync
#include <dji_sdk/SetHardSync.h>

#endif
