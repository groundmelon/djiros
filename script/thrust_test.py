#!/usr/bin/python

import rospy
from sensor_msgs.msg import Imu, Joy
import numpy as np
from tf import transformations as tfs
import math
import random

VERT_THRUST = 1.0
VERT_VELO = -1.0
GEAR_SHIFT_VALUE = -0.6


class ThrustTest:

    def __init__(self):
        self.w_q_b = None
        self.rc_data = None
        self.des_thrust = 10.0
        self.ctrl_pub = None

    def rc_callback(self, msg):
        self.rc_data = dict(roll=msg.axes[0],
                            pitch=msg.axes[1],
                            yaw=msg.axes[2],
                            thrust=msg.axes[3],
                            mode=msg.axes[4],
                            gear=msg.axes[5])

        self.process()

    def imu_callback(self, msg):
        self.w_q_b = np.array([msg.orientation.x,
                               msg.orientation.y,
                               msg.orientation.z,
                               msg.orientation.w])

    def process(self):
        if self.rc_data is None or self.w_q_b is None:
            return

        eulers = tfs.euler_from_quaternion(self.w_q_b, 'rzyx')
        yaw = eulers[0]

        # normalized to 0 ~ 1
        thr_from_rc = (self.rc_data['thrust'] + 1.0) / 2.0
        # map to 10~100
        thr_from_rc = 10.0 + 90.0 * thr_from_rc

        if (self.rc_data['gear'] < GEAR_SHIFT_VALUE):
            pass
        else:
            self.des_thrust = thr_from_rc

        rospy.loginfo("rc[{:.2f}] rcmap[{: 3.0f}] ctrl[{: 3.0f}]".format(
            self.rc_data['thrust'], thr_from_rc, self.des_thrust))

        joy_msg = Joy()
        joy_msg.header.stamp = rospy.Time.now()
        joy_msg.header.frame_id = "FRD"
        joy_msg.axes = [random.random()/100.0, random.random()/100.0, self.des_thrust, -math.degrees(yaw), VERT_THRUST]
        self.ctrl_pub.publish(joy_msg)


if __name__ == "__main__":
    rospy.init_node('thrust_test')

    thrust_test = ThrustTest()

    rc_sub = rospy.Subscriber('/djiros/rc', Joy, thrust_test.rc_callback)
    imu_sub = rospy.Subscriber('/djiros/imu', Imu, thrust_test.imu_callback)

    thrust_test.ctrl_pub = rospy.Publisher('/djiros/ctrl', Joy, queue_size=10)
    rospy.spin()
