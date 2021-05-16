// Copyright (c) 2020- Margarita Grinvald, Autonomous Systems Lab, ETH Zurich
// Licensed under the MIT License (see LICENSE for details)

#ifndef RGBD_SEGMENTATION_COMMON_H_
#define RGBD_SEGMENTATION_COMMON_H_

#include <opencv2/core/core.hpp>

struct Instance {
  int class_id;
  cv::Mat mask;
};

typedef uint32_t SegmentID;
typedef uint8_t SemanticClass;

// Segmented cloud type.
struct PointXYZRGBLNormal {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_RGB;
  uint32_t label;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRGBLNormal,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
        float, normal_y, normal_y)(float, normal_z,
                                   normal_z)(float, rgb, rgb)(uint32_t, label,
                                                              label))
// Segment pointcloud type.
struct PointXYZRGBCNormal {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_RGB;
  SemanticClass semantic_class;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRGBCNormal,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(
        float, normal_y, normal_y)(float, normal_z, normal_z)(float, rgb, rgb)(
        SemanticClass, semantic_class, semantic_class))

// Point types.
typedef PointXYZRGBLNormal InputPointType;
typedef PointXYZRGBCNormal OutputPointType;

#endif  // RGBD_SEGMENTATION_COMMON_H_
