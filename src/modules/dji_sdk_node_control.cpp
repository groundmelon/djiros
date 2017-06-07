/** @file dji_sdk_node_control.cpp
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
DJISDKNode::flightControlSubscriberCallback(
  const dji_sdk::FlightControl::ConstPtr& flight_control)
{
  Control::CtrlData ctrlData(flight_control->control_flag, flight_control->x,
                             flight_control->y, flight_control->z,
                             flight_control->yaw);
  vehicle->control->flightCtrl(ctrlData);
}

void
DJISDKNode::flightControlAdvancedSubscriberCallback(
  const dji_sdk::FlightControlAdvanced::ConstPtr& flight_control_advanced)
{
  Control::AdvancedCtrlData advCtrlData(
    flight_control_advanced->control_flag, flight_control_advanced->x,
    flight_control_advanced->y, flight_control_advanced->z,
    flight_control_advanced->yaw, flight_control_advanced->x_feedforward,
    flight_control_advanced->y_feedforward);
  vehicle->control->flightCtrl(advCtrlData);
}

void
DJISDKNode::flightControlPositionYawSubscriberCallback(
  const dji_sdk::FlightControlPosYaw::ConstPtr& flight_control_pos_yaw)
{
  vehicle->control->positionAndYawCtrl(
    flight_control_pos_yaw->x, flight_control_pos_yaw->y,
    flight_control_pos_yaw->z, flight_control_pos_yaw->yaw);
}

void
DJISDKNode::flightControlVelocityYawrateSubscriberCallback(
  const dji_sdk::FlightControlVelYawRate::ConstPtr& flight_control_vel_yawrate)
{
  vehicle->control->velocityAndYawRateCtrl(
    flight_control_vel_yawrate->Vx, flight_control_vel_yawrate->Vy,
    flight_control_vel_yawrate->Vz, flight_control_vel_yawrate->yawRate);
}

void
DJISDKNode::flightControlVelocityYawrateSubscriberCallback(
  const dji_sdk::FlightControlAttiVertPos::ConstPtr&
    flight_control_atti_vertpos)
{

  vehicle->control->attitudeAndVertPosCtrl(
    flight_control_atti_vertpos->roll, flight_control_atti_vertpos->pitch,
    flight_control_atti_vertpos->yaw, flight_control_atti_vertpos->z);
}

void
DJISDKNode::flightControlAngularrateVertposSubscriberCallback(
  const dji_sdk::FlightControlAngularRateVertPos::ConstPtr&
    flight_control_angularrate_vertpos)
{
  vehicle->control->angularRateAndVertPosCtrl(
    flight_control_angularrate_vertpos->rollRate,
    flight_control_angularrate_vertpos->yawRate,
    flight_control_angularrate_vertpos->pitchRate,
    flight_control_angularrate_vertpos->z);
}
