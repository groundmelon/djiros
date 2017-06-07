//
// Created by ltb on 6/7/17.
//

/******** RC Map (M100) *********
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
******** RC Map (A3) *********
*
*  -10000 <--->  0  <---> 10000
*   API  <---> ATT <--->  POS
*
*        CH3 +10000                     CH1 +10000
*               ^                              ^
*               |                              |                   / -5000
*    CH2        |                   CH0        |                  /
*  -10000 <-----------> +10000    -10000 <-----------> +10000    H
*               |                              |                  \
*               |                              |                   \ -10000
*               V                              V
*            -10000                         -10000
*
*
*   In this code, before publish, RC is transformed to M100 style to be compatible with controller
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

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <list>
#include <string.h>

// ros includes
#include <ros/ros.h>

// forward declaration
class DJISDKNode;

#include <dji_sdk/HardwareSync.h>

#define ASSERT_EQUALITY(x, y) ROS_ASSERT_MSG((x) == (y), "_1:%d _2:%d", (int)x, (int)y);

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

class Aligner {
public:
    // align related variables
    enum struct State_t { unaligned, aligning, aligned };

    State_t align_state;
    bool align_with_fmu;

    Aligner():align_state(State_t::unaligned), align_with_fmu(false){};

    // If no need for align, set msg_stamp and return true;
    // If need align, return false when ualigned/aligning, and return true & set msg_stamp when aligned.
    bool acquire_stamp(ros::Time& msg_stamp, uint32_t tick);

private:
    struct AlignData_t {
        ros::Time time;
        uint32_t tick;
        AlignData_t(ros::Time _time, uint32_t _tick) : time(_time), tick(_tick){};
    };

    ros::Time last_msg_stamp;
    ros::Time base_time;
    std::list<AlignData_t> alignArray;
    static int constexpr ALIGN_BUFFER_SIZE = 100;
    static double constexpr TIME_DIFF_CHECK = 0.010;
    static double constexpr TIME_DIFF_ALERT = 0.02050;
};

class DjiRos {
public:
    DjiRos(ros::NodeHandle& nh, DJISDKNode* _djisdknode);
    void process();

    Aligner aligner;

private:
    DJISDKNode* djisdknode;

public:
    std::shared_ptr<HardwareSynchronizer> m_hwsync;
    int m_hwsync_ack_count;
    double gravity;
private:
};