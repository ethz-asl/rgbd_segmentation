#include <glog/logging.h>

#include "rgbd_segmentation/rgbd_segmentation.h"

int main(int argc, char** argv) {
  std::cout << std::endl
            << "RGB-D segmentation ROS node - Copyright (c) 2020- "
               "Margarita Grinvald, Autonomous "
               "Systems Lab, ETH Zurich."
            << std::endl
            << std::endl;

  ros::init(argc, argv, "rgbd_segmentation_node");
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  RGBDSegmentation rgbd_segmentation(nh, nh_private);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
