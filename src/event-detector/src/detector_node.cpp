/**
 * @file detector_node.cpp
 *
 * @brief File containing all the detection functions
 *
 * @author Siyuan Wu
 * @date 2021-March-26
 */

/* INCLUDE FILE */
#include "detector/tracker.h"
#include "detector/motion_compensation.h"
#include "detector/obj_detector.h"
#include "ros/ros.h"

/* GLOBAL DEFINES */

using namespace tracker;

/**
 * @brief
 *
 * @param argc
 * @param argv 
 *
 * @return int
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "detector_node");
  ros::NodeHandle nh("~");

  TrackSingleObj tracker(nh);
  tracker.main();
  ROS_INFO("Hello");
  ros::spin();

  return 0;
}
