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
    bluefox2::Camera camera(nh, &djiros.sync_session);

    std::thread cam_thread;
    if (camera.isSlaveMode()) {
        cam_thread = std::thread(&bluefox2::Camera::feedSyncImages, &camera);
    } else {
        cam_thread = std::thread(&bluefox2::Camera::feedImages, &camera);
    }
     

    ros::Rate r(200.0);
    while(ros::ok()) {
        ros::spinOnce();
        djiros.process();
        r.sleep();
    }

    ROS_ERROR("[djifox] Exit...");
    cam_thread.detach();

    return 0;
}
