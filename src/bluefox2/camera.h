#include <iostream>
#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <errno.h>
#include <dynamic_reconfigure/server.h>
#include <bluefox2/on_offConfig.h>
#include <unordered_map>

#include "../HardwareSync.h"

namespace bluefox2
{


struct CameraSetting {
    std::string serial;
    std::string topic;
    
    bool use_color;
    bool use_hdr;
    bool has_hdr;
    bool use_binning;
    bool use_auto_exposure;

    int exposure_time_us;
    int auto_speed;
    
    double fps;
    double gain;
    
    bool is_slave;
};

class Camera
{
  public:
    static constexpr int MAX_CAM_CNT = 10;

    Camera(ros::NodeHandle param_nh);
    Camera(ros::NodeHandle _param_nh, SyncSession_t* ssptr);
    ~Camera();
    bool isOK();
    void feedImages();
    bool isSlaveMode();

  private:
    // Node handle
    ros::NodeHandle pnode;
    // mvIMPACT Acquire device manager
    mvIMPACT::acquire::DeviceManager devMgr;
    // create an interface to the device found
    mvIMPACT::acquire::FunctionInterface * fi[MAX_CAM_CNT];
    // establish access to the statistic properties
    mvIMPACT::acquire::Statistics * statistics[MAX_CAM_CNT];
    // Image request
    const mvIMPACT::acquire::Request *pRequest[MAX_CAM_CNT];
    
    std::unordered_map<std::string, CameraSetting> camSettings;
    std::unordered_map<std::string, int> ids;

    // Internal parameters that cannot be changed
    bool ok;
    ros::Time capture_time;
    unsigned int devCnt;
    bool m_is_slave_mode;

    // User specified parameters
    int cam_cnt;
    double m_fps;

    std::unordered_map<std::string, ros::Publisher> img_publisher;
    std::unordered_map<std::string, sensor_msgs::Image> img_buffer;

    bool initSingleMVDevice(unsigned int id, const CameraSetting& cs);
    void send_software_request();
    bool grab_image_data();

    // Hardware sync related
public:
    SyncSession_t* sync_session_ptr;
    void feedSyncImages();
private:
    bool send_hardware_request();
};

}
