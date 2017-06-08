/** @file dji_sdk_node_main.cpp
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
#include <sensor_msgs/Joy.h>
#include <stdexcept>

using namespace DJI::OSDK;

DJISDKNode::DJISDKNode() {

};

DJISDKNode::DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : telemetry_from_fc(USE_BROADCAST)
{
  // @todo need some error handling for init functions
  //! @note parsing launch file to get environment parameters
  if (!initVehicle(nh_private))
  {
    throw std::runtime_error("initVehicle failed");
  }

//  if (!initServices(nh))
//  {
//    throw std::runtime_error("initServices failed");
//  }

  if (!initFlightControl(nh_private))
  {
    throw std::runtime_error("initFlightControl failed");
  }

  if (!initSubscriber(nh_private))
  {
    throw std::runtime_error("initSubscriber failed");
  }

  if (!initPublisher(nh_private))
  {
    throw std::runtime_error("initPublisher failed");
  }
}

DJISDKNode::~DJISDKNode()
{
  cleanUpSubscribeFromFC();
  if (vehicle)
  {
    delete vehicle;
  }
}

bool
DJISDKNode::initVehicle(ros::NodeHandle& nh_private)
{
  std::string drone_version;
  std::string sDevice;
  int         baudrate;
  int         app_version;
  std::string app_bundle_id; // reserved
  int         uart_or_usb;

  nh_private.param("serial_name", sDevice, std::string("/dev/ttyUSB0"));
  nh_private.param("baud_rate", baudrate, 921600);
  nh_private.param("app_id", this->app_id, 1022384);
  nh_private.param("app_version", app_version, 1);
  nh_private.param(
    "enc_key", this->enc_key,
    std::string(
      "e7bad64696529559318bb35d0a8c6050d3b88e791e1808cfe8f7802150ee6f0d"));
  nh_private.param("uart_or_usb", uart_or_usb, 0); // chosse uart as default
  nh_private.param("drone_version", drone_version,
                   std::string("M100")); // choose M100 as default

  bool threadSupport = true;

  //! @note currently does not work without thread support
  vehicle = new Vehicle(sDevice.c_str(), baudrate, threadSupport);

  for (int i = 0; i < MAX_SUBSCRIBE_PACKAGES; i++)
    vehicle->subscribe->removePackage(i, WAIT_TIMEOUT);

  /*!
   * @note activate the drone for the user at the beginning
   *        user can also call it as a service
   *        this has been tested by giving wrong appID in launch file
   */
  if (ACK::getError(this->activate(this->app_id, this->enc_key)))
  {
    ROS_ERROR("drone activation error");
    return false;
  }
  ROS_INFO("drone activated");

  if (NULL != vehicle->subscribe)
  {
    telemetry_from_fc = USE_SUBSCRIBE;
  }

  return true;
}

// clang-format off
bool DJISDKNode::initServices(ros::NodeHandle& nh) {
  drone_activation_server   = nh.advertiseService("dji_sdk/activation",                     &DJISDKNode::droneActivationCallback,        this);
  drone_arm_server          = nh.advertiseService("dji_sdk/drone_arm_control",              &DJISDKNode::droneArmCallback,               this);
  drone_task_server         = nh.advertiseService("dji_sdk/drone_task_control",             &DJISDKNode::droneTaskCallback,              this);
  sdk_ctrlAuthority_server  = nh.advertiseService("dji_sdk/sdk_control_authority",          &DJISDKNode::sdkCtrlAuthorityCallback,       this);
  camera_action_server      = nh.advertiseService("dji_sdk/camera_action",                  &DJISDKNode::cameraActionCallback,           this);
  mfio_config_server        = nh.advertiseService("dji_sdk/mfio_config",                    &DJISDKNode::MFIOConfigCallback,             this);
  mfio_set_value_server     = nh.advertiseService("dji_sdk/mfio_set_value",                 &DJISDKNode::MFIOSetValueCallback,           this);
  waypoint_upload_server    = nh.advertiseService("dji_sdk/mission_waypoint_upload",        &DJISDKNode::missionWpUploadCallback,        this);
  waypoint_action_server    = nh.advertiseService("dji_sdk/mission_waypoint_action",        &DJISDKNode::missionWpActionCallback,        this);
  waypoint_getInfo_server   = nh.advertiseService("dji_sdk/mission_waypoint_getInfo",       &DJISDKNode::missionWpGetInfoCallback,       this);
  waypoint_getSpeed_server  = nh.advertiseService("dji_sdk/mission_waypoint_getSpeed",      &DJISDKNode::missionWpGetSpeedCallback,      this);
  waypoint_setSpeed_server  = nh.advertiseService("dji_sdk/mission_waypoint_setSpeed",      &DJISDKNode::missionWpSetSpeedCallback,      this);
  hotpoint_upload_server    = nh.advertiseService("dji_sdk/mission_hotpoint_upload",        &DJISDKNode::missionHpUploadCallback,        this);
  hotpoint_action_server    = nh.advertiseService("dji_sdk/mission_hotpoint_action",        &DJISDKNode::missionHpActionCallback,        this);
  hotpoint_getInfo_server   = nh.advertiseService("dji_sdk/mission_hotpoint_getInfo",       &DJISDKNode::missionHpGetInfoCallback,       this);
  hotpoint_setSpeed_server  = nh.advertiseService("dji_sdk/mission_hotpoint_updateYawRate", &DJISDKNode::missionHpUpdateYawRateCallback, this);
  hotpoint_resetYaw_server  = nh.advertiseService("dji_sdk/mission_hotpoint_resetYaw",      &DJISDKNode::missionHpResetYawCallback,      this);
  hotpoint_setRadius_server = nh.advertiseService("dji_sdk/mission_hotpoint_updateRadius",  &DJISDKNode::missionHpUpdateRadiusCallback,  this);
  mission_status_server     = nh.advertiseService("dji_sdkmission_status",                  &DJISDKNode::missionStatusCallback,          this);
  send_to_mobile_server     = nh.advertiseService("dji_sdk/send_data_to_mobile",            &DJISDKNode::sendToMobileCallback,           this);
  set_hardsync_server       = nh.advertiseService("dji_sdk/set_hardsyc",                    &DJISDKNode::setHardsyncCallback,            this);
  return true;
}
// clang-format on

bool
DJISDKNode::initFlightControl(ros::NodeHandle& nh)
{
  flight_control_commands = nh.subscribe<dji_sdk::FlightControl>(
    "dji_sdk/flight_control", 10, &DJISDKNode::flightControlSubscriberCallback,
    this);

  flight_control_advanced_commands =
    nh.subscribe<dji_sdk::FlightControlAdvanced>(
      "dji_sdk/flight_control_advanced", 10,
      &DJISDKNode::flightControlAdvancedSubscriberCallback, this);

  flight_control_position_yaw_commands =
    nh.subscribe<dji_sdk::FlightControlPosYaw>(
      "dji_sdk/flight_control_position_yaw", 10,
      &DJISDKNode::flightControlPositionYawSubscriberCallback, this);

  flight_control_velocity_yawrate_commands =
    nh.subscribe<dji_sdk::FlightControlVelYawRate>(
      "dji_sdk/flight_control_velocity_yawrate", 10,
      &DJISDKNode::flightControlVelocityYawrateSubscriberCallback, this);

  flight_control_attitude_vertpos_commands =
    nh.subscribe<dji_sdk::FlightControlAttiVertPos>(
      "dji_sdk/flight_control_attitude_vertpos", 10,
      &DJISDKNode::flightControlVelocityYawrateSubscriberCallback, this);

  flight_control_angularrate_vertpos_commands =
    nh.subscribe<dji_sdk::FlightControlAngularRateVertPos>(
      "dji_sdk/flight_control_angularrate_vertpos", 10,
      &DJISDKNode::flightControlAngularrateVertposSubscriberCallback, this);
  return true;
}

ACK::ErrorCode
DJISDKNode::activate(int l_app_id, std::string l_enc_key)
{
  usleep(1000000);
  Vehicle::ActivateData testActivateData;
  char                  app_key[65];
  testActivateData.encKey = app_key;
  strcpy(testActivateData.encKey, l_enc_key.c_str());
  testActivateData.ID = l_app_id;

  ROS_DEBUG("called vehicle->activate(&testActivateData, WAIT_TIMEOUT)");
  return vehicle->activate(&testActivateData, WAIT_TIMEOUT);
}

bool
DJISDKNode::initSubscriber(ros::NodeHandle& nh)
{
  gimbal_angle_cmd_subscriber = nh.subscribe<dji_sdk::Gimbal>(
    "dji_sdk/gimbal_angle_cmd", 10, &DJISDKNode::gimbalAngleCtrlCallback, this);
  gimbal_speed_cmd_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>(
    "dji_sdk/gimbal_speed_cmd", 10, &DJISDKNode::gimbalSpeedCtrlCallback, this);
  return true;
}

bool
DJISDKNode::initPublisher(ros::NodeHandle& nh)
{
  /*! rc channel output
   * axes:
   *
   *
   * */
  rc_publisher = nh.advertise<sensor_msgs::Joy>("dji_sdk/rc", 10);

  attitude_publisher =
          nh.advertise<geometry_msgs::Quaternion>("dji_sdk/attitude", 10);

  /*!
   * x: roll, rad/s
   * y: pitch, rad/s
   * z: yaw, rad/s
   * body frame
   */
  angularRate_publisher =
          nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/angular_rate", 10);

  /*!
   * TODO: decide frame
   * x: m/s^2
   * y: m/s^2
   * z: m/s^2
   */
  acceleration_publisher =
          nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/acceleration", 10);

  imu_publisher = nh.advertise<sensor_msgs::Imu>("dji_sdk/imu", 10);

  flight_status_publisher =
          nh.advertise<std_msgs::UInt8>("dji_sdk/flight_status", 10);

  gps_health_publisher =
          nh.advertise<std_msgs::UInt8>("dji_sdk/gps_health", 10);

  /*!
   * NavSatFix specs:
   *   Latitude [degrees]. Positive is north of equator; negative is south.
   *   Longitude [degrees]. Positive is east of prime meridian; negative is
   * west.
   *   Altitude [m]. Positive is above the WGS 84 ellipsoid
   */
  gps_position_publisher =
          nh.advertise<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10);

  velocity_publisher =
          nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/velocity", 10);

  from_mobile_data_publisher =
          nh.advertise<dji_sdk::MobileData>("dji_sdk/from_mobile_data", 10);

  gimbal_angle_publisher =
          nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/gimbal_angle", 10);

  ACK::ErrorCode broadcast_set_freq_ack;

  if (telemetry_from_fc == USE_BROADCAST)
  {
    ROS_INFO("Hardware or firmware only support data broadcast!");
    // default freq 50Hz
    broadcast_set_freq_ack =
            vehicle->broadcast->setBroadcastFreqDefaults(WAIT_TIMEOUT);

    if (ACK::getError(broadcast_set_freq_ack))
    {
      ACK::getErrorCodeMessage(broadcast_set_freq_ack, __func__);
      return false;
    }
    // register a callback function whenever a broadcast data is in
    vehicle->broadcast->setUserBroadcastCallback(
            &DJISDKNode::SDKBroadcastCallback, this);
  }
  else if (telemetry_from_fc == USE_SUBSCRIBE)
  {
    ROS_INFO("Hardware and firmware support data subscription!");

    // Extra topics that is only available from subscription
    displaymode_publisher =
            nh.advertise<std_msgs::UInt8>("dji_sdk/display_mode", 10);

    if (!initDataSubscribeFromFC())
    {
      return false;
    }
  }
  vehicle->moc->setFromMSDKCallback(&DJISDKNode::SDKfromMobileDataCallback,
                                    this);
  return true;
}

bool
DJISDKNode::initDataSubscribeFromFC()
{
  ACK::ErrorCode ack = vehicle->subscribe->verify(WAIT_TIMEOUT);
  if (ACK::getError(ack))
  {
    return false;
  }

  // 200 Hz package from FC
  Telemetry::TopicName topicList200Hz[] = {
          Telemetry::TOPIC_QUATERNION,
          Telemetry::TOPIC_ACCELERATION_BODY,
          Telemetry::TOPIC_ACCELERATION_GROUND,
          Telemetry::TOPIC_ACCELERATION_RAW,
          Telemetry::TOPIC_PALSTANCE_FUSIONED,
          Telemetry::TOPIC_PALSTANCE_RAW,
          Telemetry::TOPIC_VELOCITY
  };
  int nTopic200Hz    = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
  int packageID200Hz = 0;
  if (vehicle->subscribe->initPackageFromTopicList(packageID200Hz, nTopic200Hz,
                                                   topicList200Hz, 0, 200))
  {
    ack = vehicle->subscribe->startPackage(packageID200Hz, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(packageID200Hz, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 200Hz mpackage");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              packageID200Hz, publish200HzData, this);
    }
  }

  // 50 Hz package from FC
  Telemetry::TopicName topicList50Hz[] = {
          Telemetry::TOPIC_GPS_FUSED,     Telemetry::TOPIC_HEIGHT_FUSION,
          Telemetry::TOPIC_STATUS_FLIGHT, Telemetry::TOPIC_STATUS_DISPLAYMODE,
          Telemetry::TOPIC_GPS_DATE,      Telemetry::TOPIC_GPS_TIME,
          Telemetry::TOPIC_GPS_POSITION,  Telemetry::TOPIC_GPS_VELOCITY,
          Telemetry::TOPIC_GPS_DETAILS,   Telemetry::TOPIC_GIMBAL_ANGLES,
          Telemetry::TOPIC_GIMBAL_STATUS, Telemetry::TOPIC_RC
  };
  int nTopic50Hz    = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  int packageID50Hz = 1;

  if (vehicle->subscribe->initPackageFromTopicList(packageID50Hz, nTopic50Hz,
                                                   topicList50Hz, 0, 50))
  {
    ack = vehicle->subscribe->startPackage(packageID50Hz, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(packageID50Hz, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 50Hz mpackage");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              packageID50Hz, publish50HzData, (UserData) this);
    }
  }
  return true;
}

void
DJISDKNode::cleanUpSubscribeFromFC()
{
  vehicle->subscribe->removePackage(0, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(1, WAIT_TIMEOUT);
}
