#ifndef DEPTHMAPRENDERER_H
#define DEPTHMAPRENDERER_H

#include <vital/types/camera_perspective.h>
#include <arrows/core/render_mesh_depth_map.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>
#include <vital/io/mesh_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DepthMapRenderer
{
  using Point = pcl::PointXYZ;
  using Pointcloud = pcl::PointCloud<Point>;

public:
  DepthMapRenderer(std::string const& mesh_filename, double f, unsigned int width, unsigned int height);

  void setCameraPosition(kwiver::vital::vector_3d const& p) {
    camera->set_center(p);
  }

  void setCameraOrientation(kwiver::vital::rotation_d const& rot) {
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
