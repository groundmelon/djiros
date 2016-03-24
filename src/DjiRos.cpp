#include "DjiRos.h"

#define _TICK2ROSTIME(tick) (ros::Duration((double)(tick) / 400.0))

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;

DjiRos::DjiRos(ros::NodeHandle& nh)
    : align_state(AlignState_t::unaligned),
      align_with_fmu(true),
      api_trigger(
          static_cast<int16_t>(5000)),  // 8000 is F-mode, 0 is A-mode, set threshold to 5000
      ctrl_state(CtrlState_t::released),
      ctrl_cmd_wait_timeout_limit(0.0),
      ctrl_cmd_stream_timeout_limit(0.0),
      last_ctrl_stamp(ros::Time(0)),
      sdk_control_flag(false) {
    std::string serial_name;
    int baud_rate;
    int app_id;
    int app_version;
    std::string app_bundle_id;
    std::string enc_key;

    int uart_or_usb;
    int A3_or_M100;
    nh.param("serial_name", serial_name, std::string("/dev/null"));
    nh.param("baud_rate", baud_rate, 230400);
    nh.param("app_id", app_id, 0);
    nh.param("enc_key", enc_key, std::string(""));
    nh.param("app_version", app_version, 1);
    nh.param("app_bundle_id", app_bundle_id, std::string(""));

    nh.param("uart_or_usb", uart_or_usb, 0);  // chosse uart as default
    nh.param("A3_or_M100", A3_or_M100, 1);    // chosse M100 as default
    nh.param("only_broadcast", only_broadcast,
             false);  // broadcast mode, no control signal send to sdk
    nh.param("align_with_fmu", align_with_fmu,
             false);  // Where djiros will use ticks from fmu to align with it
    nh.param("gravity", gravity, 9.79);

    nh.param("ctrl_cmd_stream_timeout", ctrl_cmd_stream_timeout_limit, 0.1);
    nh.param("ctrl_cmd_wait_timeout", ctrl_cmd_wait_timeout_limit, 3.0);

    // activation
    user_act_data.ID = app_id;

    if ((uart_or_usb) && (A3_or_M100)) {
        throw std::runtime_error("M100 does not support USB API.");
    }

    if (A3_or_M100) {  // for M100
        user_act_data.version = 0x03010a00;
    } else {
        user_act_data.version = 0x03016400;
    }

    strncpy((char*)user_act_data.iosID, app_bundle_id.c_str(), 32);
    user_act_data.encKey = app_key_buffer;
    strncpy(user_act_data.encKey, enc_key.c_str(), 64 + 1);

    printf("=================================================\n");
    printf("app id: %d\n", user_act_data.ID);
    printf("app version: 0x0%X\n", user_act_data.version);
    printf("app key: %s\n", user_act_data.encKey);
    printf("=================================================\n");

    if (uart_or_usb)  // use usb port for SDK
    {
        sdk.usbHandshake(serial_name);
    }

    if (!sdk.init(serial_name, baud_rate)) {
        std::string prompt = "Failed to open " + serial_name;
        ROS_ERROR_STREAM(prompt);
        throw std::runtime_error(prompt.c_str());
    }

    // ros related initialization
    ros_R_fc << 1, 0, 0, 0, -1, 0, 0, 0, -1;

    pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 10);
    pub_velo = nh.advertise<geometry_msgs::Vector3Stamped>("velo", 10);
    pub_gps = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);
    pub_mag = nh.advertise<sensor_msgs::MagneticField>("mag", 10);
    pub_rc = nh.advertise<sensor_msgs::Joy>("rc", 10);
    pub_gimbal = nh.advertise<geometry_msgs::Vector3Stamped>("gimbal", 10);
    pub_time_ref = nh.advertise<sensor_msgs::TimeReference>("tick", 10);

    if (!only_broadcast) {
        ctrl_sub =
            nh.subscribe<sensor_msgs::Joy>("ctrl", 10, boost::bind(&DjiRos::control_callback, this, _1),
                                           ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
        gimbal_ctrl_sub = nh.subscribe<geometry_msgs::PoseStamped>(
            "gimbal_ctrl", 10, boost::bind(&DjiRos::gimbal_control_callback, this, _1),
            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
        gimbal_speed_ctrl_sub = nh.subscribe<geometry_msgs::TwistStamped>(
            "gimbal_speed_ctrl", 10, boost::bind(&DjiRos::gimbal_speed_control_callback, this, _1),
            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());
    }


    if (!only_broadcast && !sdk.activate(&user_act_data)) {
        throw std::runtime_error("Activation failed");
    };

    sdk.setBroadcastCallback(&DjiRos::on_broadcast, this);
    
    if (!only_broadcast) {
        sdk.setObtainControlCallback(&DjiRos::on_control, this);
    }
    // sdk.setFromMobileCallback(&DjiRos::transparent_transmission_callback, this);
}

DjiRos::~DjiRos() {
    if (!only_broadcast) {
        ROS_INFO("[djiros] Release on exit.");
        sdk.obtain_control(false);
        // Wait sometime for display information, but will shutdown even if sdk has wrong/no response
        ros::Duration(0.5).sleep();
    }
}

void DjiRos::process() {
    ros::Time now_time = ros::Time::now();

    if (only_broadcast) {
        return;
    }

    bool switch_into_F_mode = api_trigger.isRaiseEdge();
    bool out_of_F_mode = (api_trigger.getLevel() == 0);
    // This timeout checks if control command stops streaming in
    bool ctrl_cmd_stream_timeout =
        (now_time - last_ctrl_stamp).toSec() > ctrl_cmd_stream_timeout_limit;
    // This timeout checks if control command stops streaming in
    bool ctrl_cmd_wait_timeout =
        (now_time - wait_start_stamp).toSec() > ctrl_cmd_wait_timeout_limit;
    bool ctrl_obtained = (sdk_control_flag == 1);

    // ROS_INFO_STREAM("bools: " << switch_into_F_mode <<
    // out_of_F_mode <<
    // ctrl_cmd_stream_timeout <<
    // ctrl_cmd_wait_timeout <<
    // ctrl_obtained);

    if (ctrl_state == CtrlState_t::released) {
        if (switch_into_F_mode) {
            ctrl_state = CtrlState_t::wait_for_command;
            wait_start_stamp = now_time;
        }
    } else if (ctrl_state == CtrlState_t::wait_for_command) {
        if (out_of_F_mode) {
            manually_leave_api_mode(false);
            ctrl_state = CtrlState_t::released;
        } else {
            if (!ctrl_cmd_stream_timeout) {
                ctrl_state = CtrlState_t::obtaining;
                obtain_control(true);
            } else {
                if (ctrl_cmd_wait_timeout) {
                    ctrl_state = CtrlState_t::released;
                    ROS_WARN("[djiros] No ctrl cmd received. Exit api mode.");
                }
                else {
                    if ((now_time - wait_start_stamp).toSec() > 1.0) {
                        ROS_INFO_THROTTLE(1.0, "[djiros] Waiting for control command ...");
                    }
                }
            }
        }
    } else if (ctrl_state == CtrlState_t::obtaining) {
        if (out_of_F_mode) {
            manually_leave_api_mode(false);
            ctrl_state = CtrlState_t::released;
        } else if (ctrl_obtained) {
            ctrl_state = CtrlState_t::obtained;
        } else {
            ROS_INFO_DELAYED_THROTTLE(1.0, "[djiros] Obtaining control...");
        }
    } else if (ctrl_state == CtrlState_t::obtained) {
        if (out_of_F_mode) {
            manually_leave_api_mode(true);
            ctrl_state = CtrlState_t::released;
        } else {
            if (ctrl_cmd_stream_timeout) {
                ROS_ERROR("[djiros] Control command is stopped for [%.0f] ms! Exit api mode.",
                    (now_time - last_ctrl_stamp).toSec() * 1000.0);
                obtain_control(false);
                ctrl_state = CtrlState_t::released;
            }
        }
    }
}

void DjiRos::on_broadcast() {
    ros::Time now_time = ros::Time::now();
    DJI::onboardSDK::BroadcastData bc_data = sdk.coreAPI->getBroadcastData();
    unsigned short msg_flags = bc_data.dataFlag;

    // TODO:
    // ROS_INFO("Msg flag: %x", bc_data.dataFlag);

    ros::Time msg_stamp = now_time;

    if (align_with_fmu) {
        bool msg_can_be_used_for_align = (msg_flags & HAS_TIME) && (msg_flags & HAS_W);
        // only use 100HZ imu bag (0x3f) to align
        if (align_state == AlignState_t::unaligned && msg_can_be_used_for_align) {
            base_time = now_time - _TICK2ROSTIME(bc_data.timeStamp.time);
            align_state = AlignState_t::aligning;
            ROS_ERROR("[djiros] start imu align");
            return;
        }

        if (align_state == AlignState_t::aligning && msg_can_be_used_for_align) {
            ROS_INFO_THROTTLE(1.0, "[djiros] IMU aliging...");
            if (alignArray.size() < ALIGN_BUFFER_SIZE) {
                alignArray.emplace_back(now_time, bc_data.timeStamp.time);
            } else {
                assert(alignArray.size() == ALIGN_BUFFER_SIZE);
                bool all_sample_pass_test = true;
                for (auto it = alignArray.begin(); it != alignArray.end(); ++it) {
                    double dt = (it->time - (base_time + _TICK2ROSTIME(it->tick))).toSec();
                    if (std::fabs(dt) > TIME_DIFF_CHECK) {
                        all_sample_pass_test = false;
                        alignArray.erase(alignArray.begin(), std::next(it));
                        break;
                    }
                }

                if (all_sample_pass_test) {
                    ROS_ERROR("[djiros] ***** IMU ALIGNED *****");
                    align_state = AlignState_t::aligned;
                } else {
                    alignArray.emplace_back(now_time, bc_data.timeStamp.time);
                    base_time = now_time - _TICK2ROSTIME(alignArray.front().tick);
                }
            }
            return;
        }

        if (align_state == AlignState_t::aligned) {
            msg_stamp = base_time + _TICK2ROSTIME(bc_data.timeStamp.time);
            double dt = (now_time - msg_stamp).toSec();

            if (std::fabs(dt) > TIME_DIFF_ALERT) {
                static int cnt = 0;
                ++cnt;
                ROS_WARN_THROTTLE(1.0, "[djiros] SysTime - TickTime = %.0f ms [%d]", dt * 1000,
                                  cnt);
            }
        }
    } else  // Will not align with fmu, just use ros::Time::now()
    {
        static ros::Time last_msg_stamp;
        ROS_DEBUG("[djiros] Not align with fmu, msg-seq-interval=%.3f", (now_time - last_msg_stamp).toSec());
        msg_stamp = now_time;
        last_msg_stamp = msg_stamp;
    }

    if ((msg_flags & HAS_W) && (msg_flags & HAS_A) && (msg_flags & HAS_Q)) {
        Quaterniond q;
        q.w() = bc_data.q.q0;
        q.x() = bc_data.q.q1;
        q.y() = bc_data.q.q2;
        q.z() = bc_data.q.q3;

        Matrix3d gRb = q.toRotationMatrix();

        // accel in std_msg is the raw acceleration output from IMU
        Vector3d a_b;
        a_b(0) = bc_data.a.x;
        a_b(1) = bc_data.a.y;
        a_b(2) = bc_data.a.z;
        a_b = a_b * gravity;

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = msg_stamp;
        imu_msg.header.frame_id = std::string("FLU");
        Quaterniond q_ros(Matrix3d(ros_R_fc * gRb * ros_R_fc.transpose()));
        imu_msg.orientation.x = q_ros.x();
        imu_msg.orientation.y = q_ros.y();
        imu_msg.orientation.z = q_ros.z();
        imu_msg.orientation.w = q_ros.w();
        imu_msg.angular_velocity.x = bc_data.w.x;
        imu_msg.angular_velocity.y = -bc_data.w.y;
        imu_msg.angular_velocity.z = -bc_data.w.z;
        imu_msg.linear_acceleration.x = a_b(0);
        imu_msg.linear_acceleration.y = -a_b(1);
        imu_msg.linear_acceleration.z = -a_b(2);

        pub_imu.publish(imu_msg);
    }

    if ((msg_flags & HAS_V)) {
        geometry_msgs::Vector3Stamped velo_msg;
        if (bc_data.v.health) {
            velo_msg.header.stamp = msg_stamp;
            velo_msg.header.frame_id = std::string("NED");
            velo_msg.vector.x = bc_data.v.x;
            velo_msg.vector.y = bc_data.v.y;
            velo_msg.vector.z = -bc_data.v.z;
        }

        pub_velo.publish(velo_msg);
    }
#define GPS_TO_DEGREES 1
    if ((msg_flags & HAS_POS)) {
        double scale_factor = 1.0;
     
#if GPS_TO_DEGREES
        scale_factor = 180.0 / M_PI;
#endif
        sensor_msgs::NavSatFix gps_msg;
        gps_msg.header.stamp = msg_stamp;
        gps_msg.header.frame_id = std::string("NED");
        gps_msg.latitude = bc_data.pos.latitude * scale_factor;
        gps_msg.longitude = bc_data.pos.longitude * scale_factor;
        // gps_msg.altitude = bc_data.pos.alti; // baro height
        gps_msg.altitude = bc_data.pos.height;  // height w.r.t ground
        gps_msg.status.status = bc_data.pos.health;

        pub_gps.publish(gps_msg);
    }

    if ((msg_flags & HAS_MAG)) {
        sensor_msgs::MagneticField mag_msg;
        mag_msg.header.stamp = msg_stamp;
        mag_msg.header.frame_id = std::string("NED");
        mag_msg.magnetic_field.x = bc_data.mag.x;
        mag_msg.magnetic_field.y = bc_data.mag.y;
        mag_msg.magnetic_field.z = bc_data.mag.z;

        pub_mag.publish(mag_msg);
    }

    bool rc_is_on = true;
    // this switch cannot be zero except the rc is off
    if (bc_data.rc.gear == 0) {
        rc_is_on = false;
    }

    if ((msg_flags & HAS_RC) && rc_is_on) {
        sensor_msgs::Joy rc_msg;
        rc_msg.header.stamp = msg_stamp;
        rc_msg.header.frame_id = std::string("rc");
        rc_msg.axes.push_back(static_cast<float>(bc_data.rc.roll / 10000.0));
        rc_msg.axes.push_back(static_cast<float>(bc_data.rc.pitch / 10000.0));
        rc_msg.axes.push_back(static_cast<float>(bc_data.rc.yaw / 10000.0));
        rc_msg.axes.push_back(static_cast<float>(bc_data.rc.throttle / 10000.0));
        rc_msg.axes.push_back(static_cast<float>(bc_data.rc.mode / 10000.0));
        rc_msg.axes.push_back(static_cast<float>(bc_data.rc.gear / 10000.0));

        rc_msg.buttons.push_back(static_cast<int>(bc_data.status));
        rc_msg.buttons.push_back(static_cast<int>(bc_data.battery));
        rc_msg.buttons.push_back(static_cast<int>(bc_data.ctrlInfo.device));

        pub_rc.publish(rc_msg);

        api_trigger.setValue(bc_data.rc.mode);
    }

    if ((msg_flags & HAS_GIMBAL)) {
        geometry_msgs::Vector3Stamped gmb_msg;
        gmb_msg.header.stamp = msg_stamp;
        gmb_msg.header.frame_id = std::string("gimbal");
        gmb_msg.vector.x = bc_data.gimbal.roll;
        gmb_msg.vector.y = bc_data.gimbal.pitch;
        gmb_msg.vector.z = bc_data.gimbal.yaw;

        pub_gimbal.publish(gmb_msg);
    }

    if ((msg_flags & HAS_TIME)) {
        sensor_msgs::TimeReference tmr_msg;
        tmr_msg.header.stamp = msg_stamp;
        tmr_msg.time_ref = ros::Time(static_cast<double>(bc_data.timeStamp.time));
        tmr_msg.source = std::string("from fmu, 400 ticks/sec");

        pub_time_ref.publish(tmr_msg);
    }
}

// For meaning of event, see DjiSdkRosAdapter.h, function
// static void obtainControlCallback(CoreAPI *This, Header *header, void *userData)
void DjiRos::on_control(int event) {
    if (event == 1) {
        ROS_ERROR(ANSI_COLOR_GREEN
                  "[djiros] ****** Acquire control success! ******" ANSI_COLOR_RESET);
        sdk_control_flag = true;
    } else if (event == 0) {
        ROS_ERROR(ANSI_COLOR_CYAN
                  "[djiros] ****** Release control success! ******" ANSI_COLOR_RESET);
        sdk_control_flag = false;
    } else if (event == -1 && api_trigger.getLevel() == 0) {
        // RC is not in F mode
        ROS_ERROR(ANSI_COLOR_CYAN
                  "[djiros] ****** Control is released by RC ******" ANSI_COLOR_RESET);
        sdk_control_flag = false;
    } else if (event == -2) {
        // Running, pass
    } else {
        ROS_ERROR("[djiros] Obtain/Release control operation failed. event=%d", event);
    }
}

void DjiRos::manually_leave_api_mode(bool need_release) {
    if (need_release) {
        obtain_control(false);
    }
    ROS_ERROR(ANSI_COLOR_CYAN "[djiros] Manually exit api mode." ANSI_COLOR_RESET);
}

void DjiRos::obtain_control(bool b) {
    sdk.obtain_control(b);
}

void DjiRos::control_callback(const sensor_msgs::JoyConstPtr& pMsg) {
    if (pMsg->header.frame_id.compare("FRD") == 0) {
        last_ctrl_stamp = ros::Time::now();

        DJI::onboardSDK::FlightData flight_ctrl_data;

        flight_ctrl_data.x = pMsg->axes[0];
        flight_ctrl_data.y = pMsg->axes[1];
        flight_ctrl_data.z = pMsg->axes[2];
        flight_ctrl_data.yaw = pMsg->axes[3];
        if (pMsg->axes[4] > 0) {
            flight_ctrl_data.flag = 0b00100010;  // 0b00100000, mode 13
            // user_ctrl_data.ctrl_flag = 0b00101010; // 0b00100000, mode 14
        } else {
            flight_ctrl_data.flag = 0b00000010;  // 0b00000000, mode 1
        }

        sdk.flight->setFlight(&flight_ctrl_data);

        // if (pMsg->buttons.size()) {
        //     ros::Time feedback_stamp;
        //     feedback_stamp.sec = pMsg->buttons[1] * pMsg->buttons[0] + pMsg->buttons[2];
        //     feedback_stamp.nsec = pMsg->buttons[3] * pMsg->buttons[0] + pMsg->buttons[4];
        //     ROS_INFO("curr: %.3f fbk: %.3f dt=%.3f", last_ctrl_time.toSec(),
        //     feedback_stamp.toSec(),
        //              (last_ctrl_time - feedback_stamp).toSec());
        // }
    } else {
        ROS_ERROR("[djiros] input joy_msg.frame_id should be FRD!!!");
    }
}

void DjiRos::gimbal_control_callback(const geometry_msgs::PoseStampedConstPtr& pMsg) {
    const double angle_unit_convert_base = 0.1 / 180.0 * M_PI;  // 0.1 degrees in rads

    DJI::onboardSDK::GimbalAngleData gimbal_angle;
    gimbal_angle.yaw =
        static_cast<signed short>(-1 * pMsg->pose.orientation.z / angle_unit_convert_base);
    gimbal_angle.roll =
        static_cast<signed short>(pMsg->pose.orientation.x / angle_unit_convert_base);
    gimbal_angle.pitch =
        static_cast<signed short>(-1 * pMsg->pose.orientation.y / angle_unit_convert_base);
    gimbal_angle.duration = pMsg->pose.orientation.w;
    gimbal_angle.mode = 0xF0;
    gimbal_angle.mode &= true ? 0xFF : 0x7F;
    gimbal_angle.mode &= (pMsg->pose.position.z < 0.0) ? 0xFF : 0xBF;
    gimbal_angle.mode &= (pMsg->pose.position.x < 0.0) ? 0xFF : 0xDF;
    gimbal_angle.mode &= (pMsg->pose.position.y < 0.0) ? 0xFF : 0xEF;

    sdk.camera->setGimbalAngle(&gimbal_angle);
}

void DjiRos::gimbal_speed_control_callback(const geometry_msgs::TwistStampedConstPtr& pMsg) {
    const double angle_unit_convert_base = 0.1 / 180.0 * M_PI;  // 0.1 degrees in rads

    DJI::onboardSDK::GimbalSpeedData gimbal_speed;
    gimbal_speed.yaw =
        static_cast<signed short>(-1 * pMsg->twist.angular.z / angle_unit_convert_base);
    gimbal_speed.roll = static_cast<signed short>(pMsg->twist.angular.x / angle_unit_convert_base);
    gimbal_speed.pitch =
        static_cast<signed short>(-1 * pMsg->twist.angular.y / angle_unit_convert_base);
    gimbal_speed.reserved = 0x80;  // little endian. enable

    sdk.camera->setGimbalSpeed(&gimbal_speed);
}
