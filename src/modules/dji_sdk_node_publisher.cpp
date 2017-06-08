/** @file dji_sdk_node_publisher.cpp
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

// using namespace DJI::OSDK::Telemetry;

void
DJISDKNode::SDKBroadcastCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                 DJI::OSDK::UserData userData)
{
  ((DJISDKNode*)userData)->dataBroadcastCallback();
}

void
DJISDKNode::dataBroadcastCallback()
{
  using namespace DJI::OSDK;

  std_msgs::Header header;
  static int       seqID;
  header.frame_id = seqID++;
  header.stamp    = ros::Time::now();
  header.frame_id = "/base_link";

  short int data_enable_flag = vehicle->broadcast->getPassFlag();

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_RC)
  {
    sensor_msgs::Joy rc;
    //! @note wrong implemtation use push_back
    //    rc.axes[0] = vehicle->broadcast->getRC().throttle;
    //    rc.axes[1] = vehicle->broadcast->getRC().yaw;
    //    rc.axes[2] = vehicle->broadcast->getRC().roll;
    //    rc.axes[3] = vehicle->broadcast->getRC().pitch;
    //    rc.axes[4] = vehicle->broadcast->getRC().mode;
    //    rc.axes[5] = vehicle->broadcast->getRC().gear;
    rc_publisher.publish(rc);
  }

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_Q)
  {
    geometry_msgs::Quaternion q;
    //! @note this mapping is tested and verified
    q.w = vehicle->broadcast->getQuaternion().q0;
    q.x = vehicle->broadcast->getQuaternion().q1;
    q.y = vehicle->broadcast->getQuaternion().q2;
    q.z = vehicle->broadcast->getQuaternion().q3;
    attitude_publisher.publish(q);
  }

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_W)
  {
    geometry_msgs::Vector3Stamped angular_rate;
    angular_rate.header = header;
    //! @note this is NED frame
    angular_rate.vector.x = vehicle->broadcast->getPalstance().x;
    angular_rate.vector.y = vehicle->broadcast->getPalstance().y;
    angular_rate.vector.z = vehicle->broadcast->getPalstance().z;
    angularRate_publisher.publish(angular_rate);
  }

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_A)
  {
    geometry_msgs::Vector3Stamped acceleration;
    acceleration.header          = header;
    acceleration.header.frame_id = "/map";
    //! @note this is NEU frame
    acceleration.vector.x = vehicle->broadcast->getAcceleration().x;
    acceleration.vector.y = vehicle->broadcast->getAcceleration().y;
    acceleration.vector.z = vehicle->broadcast->getAcceleration().z;
    acceleration_publisher.publish(acceleration);
  }

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_POS)
  {
    DJI::OSDK::Telemetry::GlobalPosition global_pos =
      vehicle->broadcast->getGlobalPosition();
    std_msgs::UInt8 gps_health;
    gps_health.data = global_pos.health;
    gps_health_publisher.publish(gps_health);

    sensor_msgs::NavSatFix gps_pos;
    gps_pos.header          = header;
    gps_pos.header.frame_id = "/gps";
    gps_pos.latitude        = global_pos.latitude * 180 / C_PI;
    gps_pos.longitude       = global_pos.longitude * 180 / C_PI;
    gps_pos.altitude        = global_pos.altitude;
    gps_position_publisher.publish(gps_pos);
  }

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_V)
  {
    geometry_msgs::Vector3Stamped velocity;
    velocity.header          = header;
    velocity.header.frame_id = "/map";
    //! @note this is NEU frame
    velocity.vector.x = vehicle->broadcast->getVelocity().x;
    velocity.vector.y = vehicle->broadcast->getVelocity().y;
    velocity.vector.z = vehicle->broadcast->getVelocity().z;
    velocity_publisher.publish(velocity);
  }
}

void
DJISDKNode::publish50HzData(Vehicle* vehicle, RecvContainer recvFrame,
                            DJI::OSDK::UserData userData)
{
  DJISDKNode* p = (DJISDKNode*)userData;

  Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type latlong =
    vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
  Telemetry::TypeMap<Telemetry::TOPIC_HEIGHT_FUSION>::type height =
    vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
  sensor_msgs::NavSatFix gps_pos;
  gps_pos.header.frame_id = "/gps";
  gps_pos.latitude        = latlong.latitude * 180 / C_PI;
  gps_pos.longitude       = latlong.longitude * 180 / C_PI;
  gps_pos.altitude        = height;
  p->gps_position_publisher.publish(gps_pos);

  Telemetry::TypeMap<Telemetry::TOPIC_STATUS_FLIGHT>::type fs =
    vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
  std_msgs::UInt8 flight_status;
  flight_status.data = fs;
  p->flight_status_publisher.publish(flight_status);

  Telemetry::TypeMap<Telemetry::TOPIC_GIMBAL_ANGLES>::type gimbal_angle =
    vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>();
  geometry_msgs::Vector3Stamped gimbal_angle_vec3;
  gimbal_angle_vec3.vector.x = gimbal_angle.x;
  gimbal_angle_vec3.vector.y = gimbal_angle.y;
  gimbal_angle_vec3.vector.z = gimbal_angle.z;
  p->gimbal_angle_publisher.publish(gimbal_angle_vec3);

  Telemetry::TypeMap<Telemetry::TOPIC_STATUS_DISPLAYMODE>::type dm =
    vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();
  std_msgs::UInt8 status_dm;
  status_dm.data = dm;
  p->displaymode_publisher.publish(status_dm);

  Telemetry::TypeMap<Telemetry::TOPIC_RC>::type rc =
    vehicle->subscribe->getValue<Telemetry::TOPIC_RC>();
  sensor_msgs::Joy rc_joy;
  rc_joy.axes.reserve(6);
  rc_joy.axes.push_back(rc.throttle);
  rc_joy.axes.push_back(rc.yaw);
  rc_joy.axes.push_back(rc.roll);
  rc_joy.axes.push_back(rc.pitch);
  rc_joy.axes.push_back(rc.mode);
  rc_joy.axes.push_back(rc.gear);
  p->rc_publisher.publish(rc_joy);
}

void
DJISDKNode::publish200HzData(Vehicle* vehicle, RecvContainer recvFrame,
                             DJI::OSDK::UserData userData)
{
  DJISDKNode* p = (DJISDKNode*)userData;

  Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type quat =
    vehicle->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
  geometry_msgs::Quaternion q;
  // @note this mapping is tested
  q.w = quat.q0;
  q.x = quat.q1;
  q.y = quat.q2;
  q.z = quat.q3;
  p->attitude_publisher.publish(q);

  Telemetry::TypeMap<Telemetry::TOPIC_VELOCITY>::type v_FC =
    vehicle->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();
  geometry_msgs::Vector3Stamped v;
  // v_FC has 2 fields, data and info. The latter contains the health
  v.vector.x = v_FC.data.x;
  v.vector.y = v_FC.data.y;
  v.vector.z = v_FC.data.z;
  p->velocity_publisher.publish(v);

  Telemetry::TypeMap<Telemetry::TOPIC_PALSTANCE_FUSIONED>::type w_FC =
    vehicle->subscribe->getValue<Telemetry::TOPIC_PALSTANCE_FUSIONED>();
  geometry_msgs::Vector3Stamped angular_rate;
  angular_rate.vector.x = w_FC.x;
  angular_rate.vector.y = w_FC.y;
  angular_rate.vector.z = w_FC.z;
  p->angularRate_publisher.publish(angular_rate);

  Telemetry::TypeMap<Telemetry::TOPIC_ACCELERATION_GROUND>::type a_FC =
    vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_GROUND>();
  geometry_msgs::Vector3Stamped acceleration;
  acceleration.vector.x = a_FC.x;
  acceleration.vector.y = a_FC.y;
  acceleration.vector.z = a_FC.z;
  p->acceleration_publisher.publish(acceleration);
}
