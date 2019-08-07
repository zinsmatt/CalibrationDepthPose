#include "depthMapRenderer.h"
#include <filesystem>

namespace fs = std::filesystem;

namespace  {

void convertMeshToRegularFaces(kwiver::vital::mesh_sptr mesh)
{
  std::unique_ptr< kwiver::vital::mesh_regular_face_array<3> > regular_faces(new kwiver::vital::mesh_regular_face_array<3>);
  for (int i = 0; i < mesh->faces().size(); ++i)
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
  std::cout << "position : " <<  camera->get_center().transpose() << std::endl;
  std::cout << "orientation : \n" << camera->get_rotation().matrix() << std::endl;
  auto depthmap = renderDepthMap();
  Pointcloud::Ptr pc(new Pointcloud);
  double inv_fx = 1.0 / intrinsics->focal_length();
  double inv_fy = 1.0 / intrinsics->focal_length();
  Point p;
  for (int i = 0; i < intrinsics->image_height(); ++i)
  {
    double yn = (i - intrinsics->principal_point()[1]) * inv_fy;
    for (int j = 0; j < intrinsics->image_width(); ++j)
    {
      double xn = (j - intrinsics->principal_point()[0]) * inv_fx;
      double d = depthmap->get_image().at<double>(j, i, 0);
      if (std::isinf(d))
        continue;
//      std::cout << d << "\n";
      p.x = xn * d;
      p.y = yn * d;
      p.z = d;
      pc->push_back(p);
    }
  }
  return pc;
}
