/**
 * @file bullet_traj_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-07-16
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "bullet_traj/bullet_traj.h"
#include <iostream>
#include <ros/ros.h>

using namespace bullet_traj;

int main(int argc, char **argv) {
  ros::init(argc, argv, "bullet_traj");
  ros::NodeHandle nh("~");
  BulletCamera b_cam(nh);
  b_cam.init();
  ros::spin();
  return 0;
}
