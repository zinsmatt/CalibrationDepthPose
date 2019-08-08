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

#include "depthMapRenderer.h"
#include <filesystem>

namespace fs = std::filesystem;

namespace  {

  // Tool function to force regular faces
  void convertMeshToRegularFaces(kwiver::vital::mesh_sptr mesh)
  {
    std::unique_ptr< kwiver::vital::mesh_regular_face_array<3> > regular_faces(new kwiver::vital::mesh_regular_face_array<3>);
    for (size_t i = 0; i < mesh->faces().size(); ++i)
    {
      kwiver::vital::mesh_regular_face<3> f;
      f[0] = mesh->faces()(i, 0);
      f[1] = mesh->faces()(i, 1);
      f[2] = mesh->faces()(i, 2);
      regular_faces->push_back(f);
    }
    mesh->set_faces(std::move(regular_faces));
  }

}


DepthMapRenderer::DepthMapRenderer(const std::string &mesh_filename, double f, unsigned int width, unsigned int height) :
  intrinsics(new kwiver::vital::simple_camera_intrinsics(f, {height / 2, width / 2}, 1.0, 0.0, {}, width, height)),
  camera(new kwiver::vital::simple_camera_perspective(kwiver::vital::vector_3d::Zero(), kwiver::vital::rotation_d(), intrinsics))
{
  fs::path filePath(mesh_filename);

  if (filePath.extension() == ".obj")
  {
    mesh = kwiver::vital::read_obj(mesh_filename);
    convertMeshToRegularFaces(mesh);
  }
  else if (filePath.extension() == ".ply")
  {
    mesh = kwiver::vital::read_ply(mesh_filename);
    convertMeshToRegularFaces(mesh);
  }
  else
    std::cerr << "Unsupported mesh type (" << filePath.extension() << ")" << std::endl;
}

DepthMapRenderer::Pointcloud::Ptr DepthMapRenderer::capturePointCloud() const
{
  auto depthmap = renderDepthMap();
  Pointcloud::Ptr pc(new Pointcloud);
  double inv_f = 1.0 / intrinsics->focal_length();
  Point p;
  for (int i = 0; i < static_cast<int>(intrinsics->image_height()); ++i)
  {
    double yn = (i - intrinsics->principal_point()[1]) * inv_f;
    for (int j = 0; j < static_cast<int>(intrinsics->image_width()); ++j)
    {
      double xn = (j - intrinsics->principal_point()[0]) * inv_f;
      double d = depthmap->get_image().at<double>(j, i, 0);
      if (std::isinf(d))
        continue;
      p.x = xn * d;
      p.y = yn * d;
      p.z = d;
      pc->push_back(p);
    }
  }
  return pc;
}
