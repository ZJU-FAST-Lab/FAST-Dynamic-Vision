/**
 * @file bullet_estimation_node.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2021-07-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "bullet_traj/bullet_estimation.h"
#include <iostream>
#include <ros/ros.h>

using namespace bullet_estimation;

int main(int argc, char **argv) {
  ros::init(argc, argv, "bullet_estimation");
  ros::NodeHandle nh("~");
  BulletEst b_est(nh);
  b_est.Init();
  ros::spin();
  return 0;
}

