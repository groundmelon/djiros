#include <iostream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <signal.h>
#include <thread>

#include "DjiRos.h"
#include "bluefox2/camera.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "djifox");
    ros::NodeHandle nh("~");

    DjiRos djiros(nh);
    bluefox2::Camera camera(nh);
    std::shared_ptr<HardwareSynchronizer> hwsync(new HardwareSynchronizer());

    djiros.m_hwsync = hwsync;
    camera.m_hwsync = hwsync;

    std::thread cam_thread;
    if (camera.is_slave_mode()) {
        if (camera.is_fast_mode()) {
            ROS_WARN("[djiros/cam] camera work in fast mode");
            cam_thread = std::thread(&bluefox2::Camera::process_fast_sync, &camera);
        } else {
            cam_thread = std::thread(&bluefox2::Camera::process_slow_sync, &camera);
        }
    } else {
        cam_thread = std::thread(&bluefox2::Camera::feedImages, &camera);
    }

    ros::Rate r(200.0);
    while (ros::ok()) {
        ros::spinOnce();
        djiros.process();
        r.sleep();
    }

    ROS_ERROR("[djifox] Exit...");
    cam_thread.join();

    return 0;
}
