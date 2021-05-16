// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#include "rgbd_segmentation/rgbd_segmentation.h"

#include <chrono>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

RGBDSegmentation::RGBDSegmentation(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  std::vector<float> camera_intrinsics_vec;
  nh_private.param<std::vector<float>>(
      "camera_intrinsics", camera_intrinsics_vec, camera_intrinsics_vec);

  camera_intrinsics_ = Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(
      camera_intrinsics_vec.data());

  image_transport::ImageTransport image_transport(nh_);

  //  RGB-D subscribers.
  std::string rgb_image_sub_topic;
  nh_private.param<std::string>("rgb_image_sub_topic", rgb_image_sub_topic,
                                rgb_image_sub_topic);
  rgb_image_sub_ = new ImageSubscriber(image_transport, rgb_image_sub_topic, 1);

  std::string depth_image_sub_topic;
  nh_private.param<std::string>("depth_image_sub_topic", depth_image_sub_topic,
                                depth_image_sub_topic);
  depth_image_sub_ =
      new ImageSubscriber(image_transport, depth_image_sub_topic, 1);

  camera_info_sub_ =
      new CameraInfoSubscriber(nh_, "/camera/rgb/camera_info", 1);

  rgbd_synchronizer_ =
      new RGBDSynchronizer(RGBDSyncPolicy(10), *rgb_image_sub_,
                           *depth_image_sub_, *camera_info_sub_);
  rgbd_synchronizer_->registerCallback(
      boost::bind(&RGBDSegmentation::rgbdCallback, this, _1, _2, _3));

  float publishing_rate = 2.5f;
  nh_private_.param<float>("publishing_rate", publishing_rate, publishing_rate);
  publishing_rate_ = new ros::Rate(publishing_rate);

  // Throttled RGB-D publishers.
  throttled_rgb_image_pub_ = image_transport.advertise(
      "/rgbd_segmentation_node/rgb/image_raw", 1, false);
  throttled_depth_image_pub_ = image_transport.advertise(
      "/rgbd_segmentation_node/depth/image_raw", 1, false);
  throttled_camera_info_pub_ = nh_private_.advertise<sensor_msgs::CameraInfo>(
      "/rgbd_segmentation_node/rgb/camera_info", 1, false);

  // RGB-D segmentation subscribers.
  instance_segmentation_sub_ =
      new InstanceSegmentationSubscriber(nh_, "/mask_rcnn/result", 1000);

  cloud_segmentation_sub_ = new CloudSegmentationSubscriber(
      nh_, "/cloud_segmentation_node/segmented_cloud", 1000);

  segmentation_synchronizer_ = new SegmentationSynchronizer(
      SegmentationSyncPolicy(10), *instance_segmentation_sub_,
      *cloud_segmentation_sub_);

  segmentation_synchronizer_->registerCallback(
      boost::bind(&RGBDSegmentation::segmentationCallback, this, _1, _2));

  // RGB-D segment pointclouds publisher.
  segment_pointcloud_pub_ = nh_private_.advertise<sensor_msgs::PointCloud2>(
      "segment_pointcloud", 2000);

  // RGB-D segmentation mask publisher.
  segmentation_mask_pub_ =
      image_transport.advertise("/rgbd_segmentation_node/mask/image", 1, false);
}

void RGBDSegmentation::rgbdCallback(
    const sensor_msgs::ImageConstPtr& rgb_image,
    const sensor_msgs::ImageConstPtr& depth_image,
    const sensor_msgs::CameraInfoConstPtr& camera_info) {
  throttled_rgb_image_pub_.publish(rgb_image);
  throttled_depth_image_pub_.publish(depth_image);
  throttled_camera_info_pub_.publish(camera_info);

  publishing_rate_->sleep();
}

void RGBDSegmentation::segmentationCallback(
    const mask_rcnn_ros::Result::ConstPtr& instance_segmentation,
    const sensor_msgs::PointCloud2::ConstPtr& cloud_segmentation) {
  pcl::console::TicToc tic_toc;
  tic_toc.tic();

  pcl::PointCloud<InputPointType> segmented_cloud;
  pcl::fromROSMsg(*cloud_segmentation, segmented_cloud);

  std::vector<Instance> instances;
  extractInstances(instance_segmentation, &instances);

  publishInstanceMasks(cloud_segmentation->header.stamp,
                       cloud_segmentation->header.frame_id, instances);

  std::map<SegmentID, pcl::PointCloud<OutputPointType>> segments;
  extractSegmentPointclouds(segmented_cloud, &segments);

  std::map<size_t, std::vector<SegmentID>> instance_segments_map;
  matchSegmentsToInstances(segments, instances, &instance_segments_map);

  assignSemanticClasses(instances, instance_segments_map, &segments);

  publishSegments(cloud_segmentation->header.stamp,
                  cloud_segmentation->header.frame_id, instances,
                  instance_segments_map, &segments);
}

void RGBDSegmentation::extractInstances(
    const mask_rcnn_ros::Result::ConstPtr& instance_segmentation,
    std::vector<Instance>* instances) {
  CHECK_NOTNULL(instances);
  instances->reserve(instance_segmentation->masks.size());

  for (size_t i = 0u; i < instance_segmentation->masks.size(); ++i) {
    cv_bridge::CvImagePtr cv_mask_image = cv_bridge::toCvCopy(
        instance_segmentation->masks[i], sensor_msgs::image_encodings::MONO8);

    Instance instance;
    instance.mask = cv_mask_image->image.clone();
    instance.class_id = instance_segmentation->class_ids[i];

    instances->push_back(instance);
  }
}

void RGBDSegmentation::extractSegmentPointclouds(
    const pcl::PointCloud<InputPointType>& segmented_cloud,
    std::map<SegmentID, pcl::PointCloud<OutputPointType>>* segments) {
  CHECK_NOTNULL(segments);

  for (const auto& segment_point : segmented_cloud.points) {
    SegmentID segment_id = segment_point.label;

    PointXYZRGBCNormal point;
    point.x = segment_point.x;
    point.y = segment_point.y;
    point.z = segment_point.z;
    point.rgb = segment_point.rgb;
    point.normal_x = segment_point.normal_x;
    point.normal_y = segment_point.normal_y;
    point.normal_z = segment_point.normal_z;
    point.semantic_class = 0u;

    auto segment_it = segments->find(segment_id);
    if (segment_it != segments->end()) {
      segment_it->second.push_back(point);
    } else {
      pcl::PointCloud<OutputPointType> segment_cloud;
      segment_cloud.push_back(point);
      segments->emplace(segment_id, segment_cloud);
    }
  }
}

void RGBDSegmentation::matchSegmentsToInstances(
    const std::map<SegmentID, pcl::PointCloud<OutputPointType>>& segments,
    const std::vector<Instance>& instances,
    std::map<size_t, std::vector<SegmentID>>* instance_segments_map) {
  CHECK_NOTNULL(instance_segments_map);

  float cx = camera_intrinsics_(0, 2);
  float cy = camera_intrinsics_(1, 2);
  float fx = camera_intrinsics_(0, 0);
  float fy = camera_intrinsics_(1, 1);

  for (const auto& segment_pair : segments) {
    size_t max_overlap_count = 0u;
    size_t max_instance_id = 0u;
    size_t segment_size = segment_pair.second.points.size();

    SegmentID segment_id = segment_pair.first;

    for (size_t instance_id = 0u; instance_id < instances.size();
         ++instance_id) {
      size_t overlap_count = 0u;
      for (const auto& point : segment_pair.second.points) {
        int u = (fx * (point.x / point.z) + cx);
        int v = (fy * (point.y / point.z) + cy);

        if (instances[instance_id].mask.at<uint8_t>(v, u)) {
          ++overlap_count;
        }
      }

      if (overlap_count > max_overlap_count) {
        max_overlap_count = overlap_count;
        max_instance_id = instance_id;
      }
    }

    float overlap_ratio = (float)max_overlap_count / segment_size;

    if (max_overlap_count) {
      if (overlap_ratio > 0.8f) {
        auto instance_it = instance_segments_map->find(max_instance_id);
        if (instance_it != instance_segments_map->end()) {
          instance_it->second.push_back(segment_id);
        } else {
          std::vector<SegmentID> segment_ids;
          segment_ids.push_back(segment_id);
          instance_segments_map->emplace(max_instance_id, segment_ids);
        }
      }
    }
  }
}

void RGBDSegmentation::assignSemanticClasses(
    const std::vector<Instance>& instances,
    const std::map<size_t, std::vector<SegmentID>>& instance_segments_map,
    std::map<SegmentID, pcl::PointCloud<OutputPointType>>* segments) {
  for (const auto& instance_pair : instance_segments_map) {
    for (const SegmentID& segment_id : instance_pair.second) {
      auto segment_it = segments->find(segment_id);
      CHECK(segment_it != segments->end());
      for (OutputPointType& point : segment_it->second.points) {
        point.semantic_class = instances[instance_pair.first].class_id;
      }
    }
  }
}

void RGBDSegmentation::publishInstanceMasks(
    const ros::Time& timestamp, const std::string& frame_id,
    const std::vector<Instance>& instances) {
  if (instances.size()) {
    cv::Mat mask_union = cv::Mat::zeros(instances[0].mask.size(), CV_8U);
    for (const Instance& instance : instances) {
      cv::bitwise_or(mask_union, instance.mask, mask_union);
    }

    // Invert the mask to set to nonzero only scene parts
    // which have not been detected as an object.
    cv::bitwise_not(mask_union, mask_union);

    std_msgs::Header header;
    header.stamp = timestamp;
    header.frame_id = frame_id;
    cv_bridge::CvImage mask_image(
        header, sensor_msgs::image_encodings::TYPE_8UC1, mask_union);

    sensor_msgs::ImagePtr mask_image_msg = mask_image.toImageMsg();
    segmentation_mask_pub_.publish(mask_image_msg);
  }
}

void RGBDSegmentation::publishSegments(
    const ros::Time& timestamp, const std::string& frame_id,
    const std::vector<Instance>& instances,
    const std::map<size_t, std::vector<SegmentID>>& instance_segments_map,
    std::map<SegmentID, pcl::PointCloud<OutputPointType>>* segments) {
  // Merge all segments matched to a same instance and publish as one segment.
  for (const auto& instance_pair : instance_segments_map) {
    pcl::PointCloud<OutputPointType> instance_pointcloud;
    for (const SegmentID& segment_id : instance_pair.second) {
      auto segment_it = segments->find(segment_id);
      CHECK(segment_it != segments->end());
      instance_pointcloud += segment_it->second;
      segments->erase(segment_id);
    }

    sensor_msgs::PointCloud2 instance_pointcloud_msg;
    pcl::toROSMsg(instance_pointcloud, instance_pointcloud_msg);

    instance_pointcloud_msg.header.stamp = timestamp;
    instance_pointcloud_msg.header.frame_id = frame_id;
    segment_pointcloud_pub_.publish(instance_pointcloud_msg);
  }

  // Publish the remaining segments not matched to any instance.
  for (const auto& segment_pair : *segments) {
    sensor_msgs::PointCloud2 segment_pointcloud_msg;
    pcl::toROSMsg(segment_pair.second, segment_pointcloud_msg);

    segment_pointcloud_msg.header.stamp = timestamp;
    segment_pointcloud_msg.header.frame_id = frame_id;
    segment_pointcloud_pub_.publish(segment_pointcloud_msg);
  }
}
