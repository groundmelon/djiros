/** @file main.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  Main function for ROS Node
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

#define BACKWARD_HAS_BFD 1
#include "backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "djiros");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  auto dji_sdk_node = std::make_shared<DJISDKNode>(nh, nh_private);

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  while(ros::ok()) {
      ros::Duration(1).sleep();
  }

//  delete dji_sdk_node;
//  dji_sdk_node = NULL;

  return 0;
}
