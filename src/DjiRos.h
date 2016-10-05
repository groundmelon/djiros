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

#ifndef __DJIROS_H
#define __DJIROS_H 1

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <list>
#include <string.h>

// ros includes
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include "DjiSdkRosAdapter.h"
#include "HardwareSync.h"

#define ASSERT_EQUALITY(x, y) ROS_ASSERT_MSG((x) == (y), "_1:%d _2:%d", (int)x, (int)y);

class DataFlag_t {
  public:
    uint16_t HAS_TIME;
    uint16_t HAS_Q;
    uint16_t HAS_A;
    uint16_t HAS_V;
    uint16_t HAS_W;
    uint16_t HAS_POS;
    uint16_t HAS_GPS;
    uint16_t HAS_RTK;
    uint16_t HAS_MAG;
    uint16_t HAS_RC;
    uint16_t HAS_GMBL;
    uint16_t HAS_STAT;
    uint16_t HAS_BATT;
    uint16_t HAS_CTRL;

    DataFlag_t() : DataFlag_t(false){};

    DataFlag_t(bool is_A3)
        : HAS_TIME(0x00),
          HAS_Q(0x00),
          HAS_A(0x00),
          HAS_V(0x00),
          HAS_W(0x00),
          HAS_POS(0x00),
          HAS_GPS(0x00),
          HAS_RTK(0x00),
          HAS_MAG(0x00),
          HAS_RC(0x00),
          HAS_GMBL(0x00),
          HAS_STAT(0x00),
          HAS_BATT(0x00),
          HAS_CTRL(0x00) {
        uint16_t flag = 0x01;
        HAS_TIME = get_and_shift(flag);
        HAS_Q = get_and_shift(flag);
        HAS_A = get_and_shift(flag);
        HAS_V = get_and_shift(flag);
        HAS_W = get_and_shift(flag);
        HAS_POS = get_and_shift(flag);
        if (is_A3) {
            HAS_GPS = get_and_shift(flag);
            HAS_RTK = get_and_shift(flag);
        }
        HAS_MAG = get_and_shift(flag);
        HAS_RC = get_and_shift(flag);
        HAS_GMBL = get_and_shift(flag);
        HAS_STAT = get_and_shift(flag);
        HAS_BATT = get_and_shift(flag);
        HAS_CTRL = get_and_shift(flag);

        // for (int i = 0; i < 14; ++i) printf("%dth variable: %#x\n", i, *(((uint16_t*)&HAS_TIME) + i));
    }

  private:
    uint16_t get_and_shift(uint16_t& flag) {
        uint16_t save = flag;
        flag = flag << 1;
        return save;
    }
};

template <class T>
class LevelTrigger {
  public:
    LevelTrigger() : LevelTrigger(static_cast<T>(0)){};
    LevelTrigger(T thres_)
        : threshold(thres_), has_last_value(false), raise_trigger(false), drop_trigger(false){};

    T getValue() {
        return value;
    };

    bool getLevel() {
        return value > threshold;
    };

    // Will read and clear "Raise Trigger"
    bool isRaiseEdge() {
        if (raise_trigger) {
            raise_trigger = false;
            return true;
        } else {
            return false;
        }
    };

    // Will read and clear "Drop Trigger"
    bool isDropEdge() {
        if (drop_trigger) {
            drop_trigger = false;
            return true;
        } else {
            return false;
        }
    };

    void setValue(T x) {
        value = x;

        if (!has_last_value) {
            has_last_value = true;
            last_value = value;
            return;
        }

        if (last_value <= threshold && threshold < value) {
            raise_trigger = true;
        }

        if (last_value > threshold && threshold >= value) {
            drop_trigger = true;
        }

        last_value = value;
    };

    void clear() {
        has_last_value = false;
        raise_trigger = false;
        drop_trigger = false;
    }

  private:
    const T threshold;
    bool has_last_value;

    T value;
    T last_value;

    bool raise_trigger;
    bool drop_trigger;
};

struct AlignData_t {
    ros::Time time;
    uint32_t tick;
    AlignData_t(ros::Time _time, uint32_t _tick) : time(_time), tick(_tick){};
};

class DjiRos {
  private:
    // ros publishers and subsribers
    ros::Publisher pub_imu;
    ros::Publisher pub_velo;
    ros::Publisher pub_gps;
    ros::Publisher pub_gps_odom;
    ros::Publisher pub_mag;
    ros::Publisher pub_rc;
    ros::Publisher pub_gimbal;
    ros::Publisher pub_time_ref;
    ros::Subscriber ctrl_sub;
    ros::Subscriber gimbal_ctrl_sub;
    ros::Subscriber gimbal_speed_ctrl_sub;

  public:
    DJI::onboardSDK::DjiSdkRosAdapter sdk;

    DjiRos(ros::NodeHandle& nh);
    ~DjiRos();
    void on_broadcast();
    void process();
    void shutdown();

  private:
    ActivateData user_act_data;  // sdk is using it, so must be class variable
    char app_key_buffer[65];     // sdk is using it, so must be class variable
    double gravity;
    Eigen::Matrix3d ros_R_fc;
    bool only_broadcast;

    // align related variables
    enum struct AlignState_t { unaligned, aligning, aligned };
    AlignState_t align_state;
    bool align_with_fmu;
    ros::Time base_time;
    static int constexpr ALIGN_BUFFER_SIZE = 100;
    static double constexpr TIME_DIFF_CHECK = 0.010;
    static double constexpr TIME_DIFF_ALERT = 0.020;
    std::list<AlignData_t> alignArray;

    LevelTrigger<int16_t> api_trigger;
    DataFlag_t DTFlag;

    /* clang-format off */
    // Obtain and release control status transfer instructions
    // released             + <switch info F mode>                  = wait_for_command
    // wait_for_command     + <control command is streaming in>     = obtaining
    // obtaining            + <sdk response obtain successfully>    = obtained
    // obtained             + <control command stream timeout>      = released
    // wait_for_command     + <control command wait timeout>        = released
    /* clang-format on */
    enum struct CtrlState_t { released, wait_for_command, obtaining, obtained };
    CtrlState_t ctrl_state;
    double ctrl_cmd_wait_timeout_limit;
    double ctrl_cmd_stream_timeout_limit;
    ros::Time last_ctrl_stamp;
    ros::Time wait_start_stamp;
    bool sdk_control_flag;
    bool m_verbose_output;

    void obtain_control(bool b);
    void on_control(int event);
    void manually_leave_api_mode(bool need_release);

    // ros callbacks
    void control_callback(const sensor_msgs::JoyConstPtr& pMsg);
    void gimbal_control_callback(const geometry_msgs::PoseStampedConstPtr& pMsg);
    void gimbal_speed_control_callback(const geometry_msgs::TwistStampedConstPtr& pMsg);

public:
    std::shared_ptr<HardwareSynchronizer> m_hwsync;
private:
    int m_hwsync_ack_count;

};

#endif
