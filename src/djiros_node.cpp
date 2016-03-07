#include <iostream>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <signal.h>

#include "DjiRos.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "djiros");
    ros::NodeHandle nh("~");

    DjiRos djiros(nh);


    ros::Rate r(200.0);
    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
        djiros.process();
    }

    ROS_ERROR("[djiros] Exit...");

    return 0;
}
