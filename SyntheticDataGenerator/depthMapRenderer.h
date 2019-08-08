//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Matthieu Zins
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
//=========================================================================

#ifndef DEPTHMAPRENDERER_H
#define DEPTHMAPRENDERER_H

#include <filesystem>

#include <vital/types/camera_perspective.h>
#include <arrows/core/render_mesh_depth_map.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>
#include <vital/io/mesh_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


/**
 * @brief The DepthMapRenderer class renders depth maps of a mesh seem by a synthetic pinhole camera
 * using Kwiver algorithms. It can also directly transform the depth map into a point cloud.
 */
class DepthMapRenderer
{
  using Point = pcl::PointXYZ;
  using Pointcloud = pcl::PointCloud<Point>;

public:
  DepthMapRenderer(std::string const& mesh_filename, double f, unsigned int width, unsigned int height);

  void setCameraPosition(kwiver::vital::vector_3d const& p) {
    // This sets the position of the camera
    camera->set_center(p);
  }

  void setCameraOrientation(kwiver::vital::rotation_d const& rot) {
    // This sets the rotation (the inverse of the camera orientation)
    camera->set_rotation(rot);
  }

  kwiver::vital::image_container_sptr renderDepthMap() const {
     return kwiver::arrows::core::render_mesh_depth_map(mesh, camera);
  }

  Pointcloud::Ptr capturePointCloud() const;

private:
  kwiver::vital::mesh_sptr mesh;
  kwiver::vital::camera_intrinsics_sptr intrinsics;
  kwiver::vital::simple_camera_perspective_sptr camera;
};

#endif // DEPTHMAPRENDERER_H
