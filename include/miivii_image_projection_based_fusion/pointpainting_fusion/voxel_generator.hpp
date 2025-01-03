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

#ifndef miivii_image_projection_based_fusion__POINTPAINTING_FUSION__VOXEL_GENERATOR_HPP_
#define miivii_image_projection_based_fusion__POINTPAINTING_FUSION__VOXEL_GENERATOR_HPP_

#include <lidar_centerpoint/preprocess/voxel_generator.hpp>

#include <bitset>
#include <vector>

namespace miivii_image_projection_based_fusion
{
class VoxelGenerator : public centerpoint::VoxelGenerator
{
public:
  using centerpoint::VoxelGenerator::VoxelGenerator;

  std::size_t generateSweepPoints(std::vector<float> & points) override;
};
}  // namespace miivii_image_projection_based_fusion

#endif  // miivii_image_projection_based_fusion__POINTPAINTING_FUSION__VOXEL_GENERATOR_HPP_
