#ifndef __DJISDKROSADAPTER_H
#define __DJISDKROSADAPTER_H 1

#include "HardDriverRos.h"
#include "dji_sdk_lib/DJI_API.h"
#include "dji_sdk_lib/DJI_Flight.h"
#include "dji_sdk_lib/DJI_Camera.h"
// #include "dji_sdk_lib/DJI_VirtualRC.h"
// #include "dji_sdk_lib/DJI_WayPoint.h"
// #include "dji_sdk_lib/DJI_HotPoint.h"
// #include "dji_sdk_lib/DJI_Follow.h"

#include <ros/ros.h>
#include <string>
#include <functional>
#include <memory>
#include <thread>
#include <chrono>

namespace DJI {

namespace onboardSDK {

class DjiSdkRosAdapter {
  public:
    typedef DjiSdkRosAdapter This_t;
    DjiSdkRosAdapter() : is_inited(false), is_terminated(false) {
        sleep_duration = std::chrono::milliseconds(1);
    }

    ~DjiSdkRosAdapter() {
        is_terminated = true;
        if (worker_thread.joinable()) {
            ROS_DEBUG("joining work_thread %s:%d", __FILE__, __LINE__);
            worker_thread.join();
        }
    }

    static void activateCallback(CoreAPI *This, Header *header, void *userData) {
        CoreAPI::activateCallback(This, header, nullptr);
        ((This_t *)userData)->activation_ack_flag = true;
    }

    static void obtainControlCallback(CoreAPI *This, Header *header, void *userData) {
        unsigned short ack_data = AC_COMMON_NO_RESPONSE;

        int event = -1;
        if (header->length - EXC_DATA_SIZE <= sizeof(ack_data))
        {
            memcpy((unsigned char *)&ack_data, ((unsigned char *)header) + sizeof(Header),
                   (header->length - EXC_DATA_SIZE));
        }
        switch (ack_data)
        {
            case ACK_SETCONTROL_NEED_MODE_F:
                event = -1;
                break;
            case ACK_SETCONTROL_RELEASE_SUCCESS:
                event = 0;
                break;
            case ACK_SETCONTROL_OBTAIN_SUCCESS:
                event = 1;
                break;
            case ACK_SETCONTROL_OBTAIN_RUNNING:
            case ACK_SETCONTROL_RELEASE_RUNNING:
                event = -2;
                break;
            default:
                event = -3;
                break;
        }

        if (event == 0 || event == 1) {
            // Obtain success or release success
            ((This_t *)userData)->m_obtainControlCallback(event);
        } else if (event == -2) {
            // Running, resent commnad
            ((This_t *)userData)->obtain_control( ((This_t *)userData)->last_obtain_control_flag );
        } else {
            // Failed, provide error information to callback function
            CoreAPI::setControlCallback(This, header, nullptr);
            ((This_t *)userData)->m_obtainControlCallback(event);

        }

    }

    static void broadcastCallback(CoreAPI *coreAPI __UNUSED, Header *header __UNUSED,
                                  void *userData) {
        ((This_t *)userData)->m_broadcastCallback();
    }

    static void fromMobileCallback(CoreAPI *coreAPI __UNUSED, Header *header, void *userData) {
        uint8_t *data = ((unsigned char *)header) + sizeof(Header) + SET_CMD_SIZE;
        uint8_t len = header->length - SET_CMD_SIZE - EXC_DATA_SIZE;
        ((This_t *)userData)->m_fromMobileCallback(data, len);
    }

    static void missionStatusCallback(CoreAPI *coreAPI __UNUSED, Header *header, void *userData) {
        uint8_t *data = ((unsigned char *)header) + sizeof(Header) + SET_CMD_SIZE;
        uint8_t len = header->length - SET_CMD_SIZE - EXC_DATA_SIZE;
        ((This_t *)userData)->m_missionStatusCallback(data, len);
    }

    static void missionEventCallback(CoreAPI *coreAPI __UNUSED, Header *header, void *userData) {
        uint8_t *data = ((unsigned char *)header) + sizeof(Header) + SET_CMD_SIZE;
        uint8_t len = header->length - SET_CMD_SIZE - EXC_DATA_SIZE;
        ((This_t *)userData)->m_missionEventCallback(data, len);
    }

    bool init(std::string device, unsigned int baudrate) {
        if (is_inited) {
            return is_inited;
        }

        if (!m_hd) {
            printf("--- Connection Info ---\n");
            printf("Serial port: %s\n", device.c_str());
            printf("Baudrate: %u\n", baudrate);
            printf("-----\n");
            m_hd = std::unique_ptr<HardDriverRos>(new HardDriverRos(device, baudrate));
        } else {
            m_hd->init();
        }

        if (!m_hd->is_opened()) {
            return is_inited;
        }

        coreAPI = std::make_shared<CoreAPI>((HardDriver *)m_hd.get());
        // no log output while running hotpoint mission
        coreAPI->setHotPointData(false);

        flight = std::make_shared<Flight>(coreAPI.get());
        camera = std::make_shared<Camera>(coreAPI.get());
        // virtualRC = new VirtualRC(coreAPI);
        // waypoint = new WayPoint(coreAPI);
        // hotpoint = new HotPoint(coreAPI);
        // followme = new Follow(coreAPI);

        worker_thread = std::thread(&This_t::threadWorkerFunc, this);

        coreAPI->getVersion();

        is_inited = true;
        return is_inited;
    }

    void wait_for_ack(bool &ack_flag, std::string tips = std::string()) {
        ack_flag = false;
        while (!ack_flag) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            if (tips.size()) {
                ROS_WARN_STREAM_DELAYED_THROTTLE(1.0, tips);
            }
        }
    }

    bool activate(ActivateData *data) {
        ROS_ASSERT(is_inited);
        coreAPI->activate(data, activateCallback, (UserData) this);
        wait_for_ack(activation_ack_flag, "[djiros] Waiting for activation result...");
        return 1 == coreAPI->getBroadcastData().activation;
    }

    void obtain_control(bool b) {
        ROS_ASSERT(is_inited);
        coreAPI->setControl(b, obtainControlCallback, (UserData) this);
        last_obtain_control_flag = b;
    }

    template <class T>
    void setBroadcastCallback(void (T::*func)(), T *obj) {
        m_broadcastCallback = std::bind(func, obj);
        coreAPI->setBroadcastCallback(&This_t::broadcastCallback, (UserData) this);
    }

    template <class T>
    void setObtainControlCallback(void (T::*func)(int), T *obj) {
        m_obtainControlCallback = std::bind(func, obj, std::placeholders::_1);
        coreAPI->setBroadcastCallback(&This_t::broadcastCallback, (UserData) this);
    }

    template <class T>
    void setFromMobileCallback(void (T::*func)(uint8_t *, uint8_t), T *obj) {
        m_fromMobileCallback = std::bind(func, obj, std::placeholders::_1, std::placeholders::_2);
        coreAPI->setFromMobileCallback(&This_t::fromMobileCallback, (UserData) this);
    }

    template <class T>
    void setMissionStatusCallback(void (T::*func)(uint8_t *, uint8_t), T *obj) {
        m_missionStatusCallback =
            std::bind(func, obj, std::placeholders::_1, std::placeholders::_2);
        coreAPI->setWayPointCallback(&This_t::missionStatusCallback, (UserData) this);
    }

    template <class T>
    void setMissionEventCallback(void (T::*func)(uint8_t *, uint8_t), T *obj) {
        m_missionEventCallback = std::bind(func, obj, std::placeholders::_1, std::placeholders::_2);
        coreAPI->setWayPointEventCallback(&This_t::missionEventCallback, (UserData) this);
    }

    /*
            BroadcastData getBroadcastData() {
                return coreAPI->getBroadcastData();
            }
    */

    void sendToMobile(uint8_t *data, uint8_t len) {
        ROS_ASSERT(is_inited);
        coreAPI->sendToMobile(data, len, NULL, NULL);
    }

    void usbHandshake(std::string &device) {
        m_hd->usbHandshake(device);
    }

    // pointer to the lib API
    std::shared_ptr<CoreAPI> coreAPI;
    std::shared_ptr<Flight> flight;
    std::shared_ptr<Camera> camera;
    // VirtualRC *virtualRC;
    // WayPoint *waypoint;
    // HotPoint *hotpoint;
    // Follow *followme;

  private:
    bool is_inited;
    bool is_terminated;

    bool activation_ack_flag;
    bool last_obtain_control_flag;

    std::unique_ptr<HardDriverRos> m_hd;

    std::chrono::microseconds sleep_duration;

    std::thread worker_thread;

    std::function<void()> m_broadcastCallback;

    std::function<void(int)> m_obtainControlCallback;

    std::function<void(uint8_t *, uint8_t)> m_fromMobileCallback;

    std::function<void(uint8_t *, uint8_t)> m_missionStatusCallback;

    std::function<void(uint8_t *, uint8_t)> m_missionEventCallback;

    void threadWorkerFunc() {
        ROS_INFO_STREAM("work_thread pid=" << std::this_thread::get_id() << " is up. ");
        while (!this->is_terminated) {
            std::this_thread::sleep_for(sleep_duration);
            this->coreAPI->readPoll();
            this->coreAPI->sendPoll();
        }
        ROS_INFO_STREAM("work_thread pid=" << std::this_thread::get_id() << " is down.");
    }
};

}  // namespace onboardSDK

}  // namespace dji

#endif
