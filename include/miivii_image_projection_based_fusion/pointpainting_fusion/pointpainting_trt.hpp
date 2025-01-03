// Copyright 2022 TIER IV, Inc.
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

#ifndef miivii_image_projection_based_fusion__POINTPAINTING_FUSION__POINTPAINTING_TRT_HPP_
#define miivii_image_projection_based_fusion__POINTPAINTING_FUSION__POINTPAINTING_TRT_HPP_

#include <miivii_image_projection_based_fusion/pointpainting_fusion/voxel_generator.hpp>
#include <lidar_centerpoint/centerpoint_trt.hpp>

#include <memory>
#include <string>

namespace miivii_image_projection_based_fusion
{
static constexpr size_t CAPACITY_POINT = 1000000;
class PointPaintingTRT : public centerpoint::CenterPointTRT
{
public:
  using centerpoint::CenterPointTRT::CenterPointTRT;

  explicit PointPaintingTRT(
    const centerpoint::NetworkParam & encoder_param, const centerpoint::NetworkParam & head_param,
    const centerpoint::DensificationParam & densification_param,
    const centerpoint::CenterPointConfig & config);

protected:
  bool preprocess(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
    const tf2_ros::Buffer & tf_buffer) override;

  std::unique_ptr<miivii_image_projection_based_fusion::VoxelGenerator> vg_ptr_pp_{nullptr};
};
}  // namespace miivii_image_projection_based_fusion

#endif  // miivii_image_projection_based_fusion__POINTPAINTING_FUSION__POINTPAINTING_TRT_HPP_
