/** @file dji_sdk_node.h
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  Initializes all Publishers, Services and Actions
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

#ifndef DJI_SDK_NODE_MAIN_H
#define DJI_SDK_NODE_MAIN_H

#include <dji_sdk/dji_sdk.h>
#include <ros/ros.h>

using namespace DJI::OSDK;

class DJISDKNode
{
public:
  DJISDKNode();
  DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  DJISDKNode(const DJISDKNode& other) = delete;
  ~DJISDKNode();

  enum TELEMETRY_TYPE
  {
    USE_BROADCAST = 0,
    USE_SUBSCRIBE = 1
  };

protected:
  bool initVehicle(ros::NodeHandle& nh_private);
  bool initServices(ros::NodeHandle& nh);
  bool initFlightControl(ros::NodeHandle& nh);
  bool initSubscriber(ros::NodeHandle& nh);
  bool initPublisher(ros::NodeHandle& nh);
  bool initDataSubscribeFromFC();
  void cleanUpSubscribeFromFC();
  /*!
   * @note this function exists here instead of inside the callback function
   *        due to the usages, i.e. we not only provide service call but also
   *        call it for the user when this node was instantiated
   *        we cannot call a service without serviceClient, which is in another
   * node
   */
  ACK::ErrorCode activate(int l_app_id, std::string l_enc_key);
  //! flight control subscriber callbacks
  void flightControlSubscriberCallback(
    const dji_sdk::FlightControl::ConstPtr& flight_control);
  void flightControlAdvancedSubscriberCallback(
    const dji_sdk::FlightControlAdvanced::ConstPtr& flight_control_advanced);
  void flightControlPositionYawSubscriberCallback(
    const dji_sdk::FlightControlPosYaw::ConstPtr& flight_control_pos_yaw);
  void flightControlVelocityYawrateSubscriberCallback(
    const dji_sdk::FlightControlVelYawRate::ConstPtr&
      flight_control_vel_yawrate);
  void flightControlVelocityYawrateSubscriberCallback(
    const dji_sdk::FlightControlAttiVertPos::ConstPtr&
      flight_control_atti_vertpos);
  void flightControlAngularrateVertposSubscriberCallback(
    const dji_sdk::FlightControlAngularRateVertPos::ConstPtr&
      flight_control_angularrate_vertpos);
  //! general subscriber callbacks
  void gimbalAngleCtrlCallback(const dji_sdk::Gimbal::ConstPtr& msg);
  void gimbalSpeedCtrlCallback(
    const geometry_msgs::Vector3Stamped::ConstPtr& msg);

  //! general service callbacks
  bool droneActivationCallback(dji_sdk::Activation::Request&  request,
                               dji_sdk::Activation::Response& response);
  bool sdkCtrlAuthorityCallback(
    dji_sdk::SDKControlAuthority::Request&  request,
    dji_sdk::SDKControlAuthority::Response& response);
  //! control service callbacks
  bool droneArmCallback(dji_sdk::DroneArmControl::Request&  request,
                        dji_sdk::DroneArmControl::Response& response);
  bool droneTaskCallback(dji_sdk::DroneTaskControl::Request&  request,
                         dji_sdk::DroneTaskControl::Response& response);

  //! Mobile Data Service
  bool sendToMobileCallback(dji_sdk::SendMobileData::Request&  request,
                            dji_sdk::SendMobileData::Response& response);
  bool cameraActionCallback(dji_sdk::CameraAction::Request&  request,
                            dji_sdk::CameraAction::Response& response);
  //! mfio service callbacks
  bool MFIOConfigCallback(dji_sdk::MFIOConfig::Request&  request,
                          dji_sdk::MFIOConfig::Response& response);
  bool MFIOSetValueCallback(dji_sdk::MFIOSetValue::Request&  request,
                            dji_sdk::MFIOSetValue::Response& response);
  //! mission service callbacks
  // mission manager
  bool missionStatusCallback(dji_sdk::MissionStatus::Request&  request,
                             dji_sdk::MissionStatus::Response& response);
  // waypoint mission
  bool missionWpUploadCallback(dji_sdk::MissionWpUpload::Request&  request,
                               dji_sdk::MissionWpUpload::Response& response);
  bool missionWpActionCallback(dji_sdk::MissionWpAction::Request&  request,
                               dji_sdk::MissionWpAction::Response& response);
  bool missionWpGetInfoCallback(dji_sdk::MissionWpGetInfo::Request&  request,
                                dji_sdk::MissionWpGetInfo::Response& response);
  bool missionWpGetSpeedCallback(
    dji_sdk::MissionWpGetSpeed::Request&  request,
    dji_sdk::MissionWpGetSpeed::Response& response);
  bool missionWpSetSpeedCallback(
    dji_sdk::MissionWpSetSpeed::Request&  request,
    dji_sdk::MissionWpSetSpeed::Response& response);
  // hotpoint mission
  bool missionHpUploadCallback(dji_sdk::MissionHpUpload::Request&  request,
                               dji_sdk::MissionHpUpload::Response& response);
  bool missionHpActionCallback(dji_sdk::MissionHpAction::Request&  request,
                               dji_sdk::MissionHpAction::Response& response);
  bool missionHpGetInfoCallback(dji_sdk::MissionHpGetInfo::Request&  request,
                                dji_sdk::MissionHpGetInfo::Response& response);
  bool missionHpUpdateYawRateCallback(
    dji_sdk::MissionHpUpdateYawRate::Request&  request,
    dji_sdk::MissionHpUpdateYawRate::Response& response);
  bool missionHpResetYawCallback(
    dji_sdk::MissionHpResetYaw::Request&  request,
    dji_sdk::MissionHpResetYaw::Response& response);
  bool missionHpUpdateRadiusCallback(
    dji_sdk::MissionHpUpdateRadius::Request&  request,
    dji_sdk::MissionHpUpdateRadius::Response& response);
  //! hard sync service callback
  bool setHardsyncCallback(dji_sdk::SetHardSync::Request&  request,
                           dji_sdk::SetHardSync::Response& response);
  //! data broadcast callback
  void dataBroadcastCallback();
  void fromMobileDataCallback(RecvContainer recvFrame);
  static void SDKfromMobileDataCallback(Vehicle*            vehicle,
                                        RecvContainer       recvFrame,
                                        DJI::OSDK::UserData userData);
  static void SDKBroadcastCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                   DJI::OSDK::UserData userData);
  static void publish50HzData(Vehicle* vehicle, RecvContainer recvFrame,
                              DJI::OSDK::UserData userData);
  static void publish200HzData(Vehicle* vehicle, RecvContainer recvFrame,
                               DJI::OSDK::UserData userData);

protected:
    //! OSDK core
  Vehicle* vehicle;
    //! general service servers
  ros::ServiceServer drone_activation_server;
  ros::ServiceServer sdk_ctrlAuthority_server;
  ros::ServiceServer camera_action_server;
  //! flight control service servers
  ros::ServiceServer drone_arm_server;
  ros::ServiceServer drone_task_server;
  //! mfio service servers
  ros::ServiceServer mfio_config_server;
  ros::ServiceServer mfio_set_value_server;
  //! mission service servers
  // mission manager
  ros::ServiceServer mission_status_server;
  // waypoint mission
  ros::ServiceServer waypoint_upload_server;
  ros::ServiceServer waypoint_action_server;
  ros::ServiceServer waypoint_getInfo_server;
  ros::ServiceServer waypoint_getSpeed_server;
  ros::ServiceServer waypoint_setSpeed_server;
  // hotpoint mission
  ros::ServiceServer hotpoint_upload_server;
  ros::ServiceServer hotpoint_action_server;
  ros::ServiceServer hotpoint_getInfo_server;
  ros::ServiceServer hotpoint_setSpeed_server;
  ros::ServiceServer hotpoint_resetYaw_server;
  ros::ServiceServer hotpoint_setRadius_server;

  ros::ServiceServer send_to_mobile_server;
  //! hardsync service
  ros::ServiceServer set_hardsync_server;

  //! flight control subscribers
  ros::Subscriber flight_control_commands;
  ros::Subscriber flight_control_advanced_commands;
  ros::Subscriber flight_control_position_yaw_commands;
  ros::Subscriber flight_control_velocity_yawrate_commands;
  ros::Subscriber flight_control_attitude_vertpos_commands;
  ros::Subscriber flight_control_angularrate_vertpos_commands;
  //! general subscribers
  ros::Subscriber gimbal_angle_cmd_subscriber;
  ros::Subscriber gimbal_speed_cmd_subscriber;
  //! telemetry data publisher
  ros::Publisher attitude_publisher;
  ros::Publisher angularRate_publisher;
  ros::Publisher acceleration_publisher;
  ros::Publisher imu_publisher;
  ros::Publisher flight_status_publisher;
  ros::Publisher gps_health_publisher;
  ros::Publisher gps_position_publisher;
  ros::Publisher velocity_publisher;
  ros::Publisher from_mobile_data_publisher;
  ros::Publisher gimbal_angle_publisher;
  ros::Publisher displaymode_publisher;
  ros::Publisher rc_publisher;
  //! flight control data
  dji_sdk::FlightControl         FlightControl;
  dji_sdk::FlightControlAdvanced FlightControlAdvanced;
  //! constant
  const int WAIT_TIMEOUT           = 10;
  const int MAX_SUBSCRIBE_PACKAGES = 5;
  //! activation data
  int         app_id;
  std::string enc_key;
  //! flight controller flags
  TELEMETRY_TYPE telemetry_from_fc;
  bool           use_data_subscribe_flag;
};

#endif // DJI_SDK_NODE_MAIN_H
