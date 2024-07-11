// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef miivii_image_projection_based_fusion__SEGMENTATION_POINTCLOUD_FUSION__NODE_HPP_
#define miivii_image_projection_based_fusion__SEGMENTATION_POINTCLOUD_FUSION__NODE_HPP_

#include "miivii_image_projection_based_fusion/fusion_node.hpp"

#include <string>
#include <vector>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace miivii_image_projection_based_fusion
{
class SegmentPointCloudFusionNode : public FusionNode<PointCloud2, PointCloud2, Image>
{
private:
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pointcloud_ptr_;
  std::vector<bool> filter_semantic_label_target_;
  float filter_distance_threshold_;
  /* data */
public:
  explicit SegmentPointCloudFusionNode(const rclcpp::NodeOptions & options);

protected:
  void preprocess(PointCloud2 & pointcloud_msg) override;

  void postprocess(PointCloud2 & pointcloud_msg) override;

  void fuseOnSingleImage(
    const PointCloud2 & input_pointcloud_msg, const std::size_t image_id, const Image & input_mask,
    const CameraInfo & camera_info, PointCloud2 & output_pointcloud_msg) override;

  bool out_of_scope(const PointCloud2 & filtered_cloud);
};

}  // namespace miivii_image_projection_based_fusion
#endif  // miivii_image_projection_based_fusion__SEGMENTATION_POINTCLOUD_FUSION__NODE_HPP_
