// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef RGBD_SEGMENTATION_RGB_SEGMENTATION_H_
#define RGBD_SEGMENTATION_RGB_SEGMENTATION_H_

#include <atomic>

#include <image_transport/subscriber_filter.h>
#include <mask_rcnn_ros/Result.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include "rgbd_segmentation/common.h"

class RGBDSegmentation {
 public:
  RGBDSegmentation(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);

 private:
  typedef image_transport::SubscriberFilter ImageSubscriber;
  typedef message_filters::Subscriber<sensor_msgs::CameraInfo>
      CameraInfoSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
      RGBDSyncPolicy;
  typedef message_filters::Synchronizer<RGBDSyncPolicy> RGBDSynchronizer;

  typedef message_filters::Subscriber<mask_rcnn_ros::Result>
      InstanceSegmentationSubscriber;
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2>
      CloudSegmentationSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<
      mask_rcnn_ros::Result, sensor_msgs::PointCloud2>
      SegmentationSyncPolicy;
  typedef message_filters::Synchronizer<SegmentationSyncPolicy>
      SegmentationSynchronizer;

  void rgbdCallback(const sensor_msgs::ImageConstPtr& rgb_image,
                    const sensor_msgs::ImageConstPtr& depth_image,
                    const sensor_msgs::CameraInfoConstPtr& camera_info);

  void segmentationCallback(
      const mask_rcnn_ros::Result::ConstPtr& instance_segmentation,
      const sensor_msgs::PointCloud2::ConstPtr& cloud_segmentation);

  void extractInstances(
      const mask_rcnn_ros::Result::ConstPtr& instance_segmentation,
      std::vector<Instance>* instances);

  // Split the segmented cloud in multiple segments.
  void extractSegmentPointclouds(
      const pcl::PointCloud<InputPointType>& segmented_cloud,
      std::map<SegmentID, pcl::PointCloud<OutputPointType>>* segments);

  // Identify for each segment the corresponding
  // maximally overlapping mask, if any.
  void matchSegmentsToInstances(
      const std::map<SegmentID, pcl::PointCloud<OutputPointType>>& segments,
      const std::vector<Instance>& instances,
      std::map<size_t, std::vector<SegmentID>>* instance_segments_map);

  void assignSemanticClasses(
      const std::vector<Instance>& instances,
      const std::map<size_t, std::vector<SegmentID>>& instance_segments_map,
      std::map<SegmentID, pcl::PointCloud<OutputPointType>>* segments);

  // Publishes a binary mask that can be used for filtering out potentially
  // dynamic parts during camera tracking. It performs a bitwise OR over all
  // the predicted instance masks in the current frame and then inverts the
  // result.
  void publishInstanceMasks(const ros::Time& timestamp,
                            const std::string& frame_id,
                            const std::vector<Instance>& instances);

  // Publish segments after merging them by instance.
  void publishSegments(
      const ros::Time& timestamp, const std::string& frame_id,
      const std::vector<Instance>& instances,
      const std::map<size_t, std::vector<SegmentID>>& instance_segments_map,
      std::map<SegmentID, pcl::PointCloud<OutputPointType>>* segments);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Input RGB-D intrinsics;
  Eigen::Matrix3f camera_intrinsics_;

  // Input RGB-D subscibers.
  ImageSubscriber* rgb_image_sub_;
  ImageSubscriber* depth_image_sub_;
  CameraInfoSubscriber* camera_info_sub_;

  // Input RGB-D synchronizer.
  RGBDSynchronizer* rgbd_synchronizer_;

  // Throttled RGB-D publishers.
  image_transport::Publisher throttled_rgb_image_pub_;
  image_transport::Publisher throttled_depth_image_pub_;
  ros::Publisher throttled_camera_info_pub_;

  // Throttled RGB-D publishing rate.
  ros::Rate* publishing_rate_;

  // RGB-D segmentation subscribers.
  InstanceSegmentationSubscriber* instance_segmentation_sub_;
  CloudSegmentationSubscriber* cloud_segmentation_sub_;

  // RGB-D segmentation synchronizer.
  SegmentationSynchronizer* segmentation_synchronizer_;

  // RGB-D segment pointclouds publisher.
  ros::Publisher segment_pointcloud_pub_;

  // RGB-D segmentation mask publisher.
  image_transport::Publisher segmentation_mask_pub_;
};

#endif  // RGBD_SEGMENTATION_RGB_SEGMENTATION_H_
