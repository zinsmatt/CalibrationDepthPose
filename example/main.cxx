 
#include <iostream>
#include <memory>
#include <cmath>

#include <vtkSmartPointer.h>

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkBMPWriter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkImageShiftScale.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPLYReader.h>
#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkCamera.h>
#include <vtkRendererCollection.h>
#include <vtkOBJReader.h>
#include <vtkMatrix4x4.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>


#define PI 3.14159265359
#define TO_DEGREES(x) x * 57.2957795131
#define TO_RADIANS(x) x * 0.01745329251

#include <vital/types/camera_perspective.h>
#include <arrows/core/render_mesh_depth_map.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>
#include <vital/io/mesh_io.h>

#include "depthMapRenderer.h"

// deleter struct used for smart pointers of array
template <typename T>
struct array_deleter
{
  void operator ()(T const * p)
  {
    delete[] p;
  }
};

struct CameraIntrinsics
{
    std::size_t width, height;
    double fx, fy, cx, cy;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloudFromDepthmap(kwiver::vital::image_container_sptr depthmap, const CameraIntrinsics &intrinsic,
                                                           double near, double far)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
  double inv_fx = 1.0 / intrinsic.fx;
  double inv_fy = 1.0 / intrinsic.fy;
  pcl::PointXYZ p;
  int idx = 0;
  for (int i = 0; i < intrinsic.height; ++i)
  {
    double yn = (i - intrinsic.cy) * inv_fy;
    for (int j = 0; j < intrinsic.width; ++j)
    {
      double xn = (j - intrinsic.cx) * inv_fx;
      double d = depthmap->get_image().at<double>(i, j, 0);
      if (std::isinf(d))
        continue;
      std::cout << d << "\n";
      p.x = xn * d;
      p.y = yn * d;
      p.z = d;
      pc->push_back(p);
    }
  }
  return pc;
}



// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static KeyPressInteractorStyle* New();
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

    virtual void OnKeyPress()
    {
      // Get the keypress
      vtkRenderWindowInteractor *rwi = this->Interactor;
      std::string key = rwi->GetKeySym();

      // Output the key that was pressed
      std::cout << "Pressed " << key << std::endl;

      double *q = rwi->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetOrientationWXYZ();

      Eigen::AngleAxisd rotation(TO_RADIANS(q[0]), Eigen::Vector3d(q[1], q[2], q[3]));

      double x = -99;
      double y = -99;
      double z = -99;

      rwi->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetPosition(x, y, z);


//      rwi->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->Modified();
//      vtkMatrix4x4 *m = rwi->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetViewTransformMatrix();
//      this->CurrentRenderer->GetActiveCamera()->Modified();
//      vtkMatrix4x4 *m = this->CurrentRenderer->GetActiveCamera()->GetModelViewTransformMatrix();
//      std::cout << *(this->CurrentRenderer->GetActiveCamera()) << std::endl;
//      Eigen::Matrix4d temp;
//      for (int i = 0; i < 4; ++i)
//      {
//        for (int j = 0; j < 4; ++j)
//        {
//          temp(i, j) = m->GetElement(i, j);
//        }
//      }
//      temp = temp.inverse();
//      std::cout << temp << std::endl;


      if (dr)
      {
//        std::cout << "position " << x << " " << y << " " << z << "\n";
//        std::cout << "rotation \n " << rotation.toRotationMatrix() << "\n";

        dr->setCameraPosition(Eigen::Vector3d(x, y, z));
        dr->setCameraOrientation(Eigen::Quaterniond(0, 1, 0, 0) * Eigen::Quaterniond(rotation.toRotationMatrix()));
        auto pc = dr->capturePointCloud();
        pcl::io::savePLYFile("test.ply", *pc);
      }

      // Forward events
      vtkInteractorStyleTrackballCamera::OnKeyPress();
    }

    void setDepthMapRenderer(DepthMapRenderer* pt) {
      dr = pt;
    }

private:
    DepthMapRenderer* dr = nullptr;

};
vtkStandardNewMacro(KeyPressInteractorStyle);

int main()
{
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  vtkSmartPointer<vtkActor> actor =
    vtkSmartPointer<vtkActor>::New();
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renWin =
    vtkSmartPointer<vtkRenderWindow>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkOBJReader> fileReader =
    vtkSmartPointer<vtkOBJReader>::New();
  vtkSmartPointer<vtkWindowToImageFilter> filter =
    vtkSmartPointer<vtkWindowToImageFilter>::New();
  vtkSmartPointer<vtkBMPWriter> imageWriter =
    vtkSmartPointer<vtkBMPWriter>::New();
  vtkSmartPointer<vtkImageShiftScale> scale =
    vtkSmartPointer<vtkImageShiftScale>::New();




  vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
  camera->SetPosition(0, -4, 0);
  camera->SetFocalPoint(0, 0, 0);
  camera->SetViewUp(0, 0, 1);
  camera->SetViewAngle(60);
  double nearPlane = 0.1;
  double farPlane = 10.0;
  camera->SetClippingRange(nearPlane, farPlane);
  double angle = TO_RADIANS(camera->GetViewAngle());
  double dist = camera->GetDistance();
  camera->Modified();

    // Read .vtp file
    fileReader->SetFileName("/home/matthieu/dev/CalibrationDepthPose/example/cube_table.obj");

    //Build visualization enviroment
    mapper->SetInputConnection(fileReader->GetOutputPort());
    actor->SetMapper(mapper);
    renderer->AddActor(actor);
    renderer->SetActiveCamera(camera);
    renWin->AddRenderer(renderer);
    interactor->SetRenderWindow(renWin);
    renWin->Render();

    // Create Depth Map
    filter->SetInput(renWin);
    filter->SetMagnification(1);
    filter->SetInputBufferTypeToZBuffer();        //Extract z buffer value


    CameraIntrinsics intrinsics;
    intrinsics.height = renderer->GetSize()[0];
    intrinsics.width = renderer->GetSize()[1];
    intrinsics.cx = static_cast<double>(intrinsics.width) / 2;
    intrinsics.cy = static_cast<double>(intrinsics.height) / 2;
    intrinsics.fx = static_cast<double>(intrinsics.width)  / (2 * std::tan(angle / 2));
    intrinsics.fy = static_cast<double>(intrinsics.height) / (2 * std::tan(angle / 2));
  std::cout << "w h = " << intrinsics.width << " " << intrinsics.height << std::endl;


    vtkSmartPointer<KeyPressInteractorStyle> style =
       vtkSmartPointer<KeyPressInteractorStyle>::New(); //like paraview

    DepthMapRenderer dr("/home/matthieu/dev/CalibrationDepthPose/example/cube_table.obj", intrinsics.fx, intrinsics.width, intrinsics.height);
    style->setDepthMapRenderer(&dr);



    vtkSmartPointer<vtkAxesActor> axes =
      vtkSmartPointer<vtkAxesActor>::New();
    vtkSmartPointer<vtkOrientationMarkerWidget> widget =
      vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    widget->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
    widget->SetOrientationMarker( axes );
    widget->SetInteractor( interactor );
    widget->SetViewport( 0.0, 0.0, 0.4, 0.4 );
    widget->SetEnabled( 1 );
    widget->InteractiveOn();

     interactor->SetInteractorStyle(style);
    interactor->Start();


    filter->Modified();
    filter->Update();
    auto* imageData = filter->GetOutput();
    std::cout << *imageData << std::endl;
    vtkFloatArray* array = vtkFloatArray::SafeDownCast(imageData->GetPointData()->GetArray("ImageScalars"));
    std::cout << *array << "\n";



    int* dims = imageData->GetDimensions();
    unsigned int width = dims[0];
    unsigned int height = dims[1];

//    std::cout << "w h " << width << " " << height << std::endl;
//    std::shared_ptr<float[]> depthmap(new float[array->GetNumberOfComponents()]);
//    for (unsigned int i = 0; i < height; ++i)
//    {
//      for (unsigned int j = 0; j < width; ++j)
//      {
//        depthmap[i * width + j] = array->GetTuple1(i * width + j);
//      }
//    }


//  std::cout << "fx fy " << intrinsics.fx << " " << intrinsics.fy << std::endl;


//    kwiver::vital::camera_intrinsics_sptr camera_intrinsic(new kwiver::vital::simple_camera_intrinsics(intrinsics.fx, {intrinsics.cx, intrinsics.cy}, 1.0, 0.0, {}, width, height));


//    Eigen::Vector3d position(5, -5, 5);
//    Eigen::Quaterniond orientation(0.3535534, -0.8535534, -0.3535534, 0.1464466);
//    kwiver::vital::camera_perspective_sptr cam(new kwiver::vital::simple_camera_perspective(position, kwiver::vital::rotation_d(orientation.inverse()), camera_intrinsic));


//    auto mesh = kwiver::vital::read_obj("/home/matthieu/dev/CalibrationDepthPose/example/cube_table.obj");
//    std::unique_ptr< kwiver::vital::mesh_regular_face_array<3> > regular_faces(new kwiver::vital::mesh_regular_face_array<3>);
//    for (int i = 0; i < mesh->faces().size(); ++i)
//    {
//      kwiver::vital::mesh_regular_face<3> f;
//      f[0] = mesh->faces()(i, 0);
//      f[1] = mesh->faces()(i, 1);
//      f[2] = mesh->faces()(i, 2);
//      regular_faces->push_back(f);
//    }
//    mesh->set_faces(std::move(regular_faces));
//    std::cout << "before read ply" << std::endl;

//    std::cout << "faces regularity = " << mesh->faces().regularity()  << std::endl;
//    kwiver::vital::image_container_sptr depth = kwiver::arrows::core::render_mesh_depth_map(mesh, cam);
//    for (int i = 0; i < depth->height(); ++i)
//    {
//      for (int j= 0; j < depth->width(); ++j)
//      {
//        std::cout << depth->get_image().at<double>(i, j, 0) << std::endl;
//      }
//    }

//    pcl::PointCloud<pcl::PointXYZ>::Ptr pc = pointcloudFromDepthmap(depth, intrinsics, nearPlane, farPlane);
//    pcl::io::savePLYFile("test.ply", *pc);


//        kwiver::arrows::core::render_mesh_depth_map()

//    scale->SetOutputScalarTypeToUnsignedChar();
//    scale->SetInputConnection(filter->GetOutputPort());
//    scale->SetInputData(imageData);

//    scale->SetShift(0);
//    scale->SetScale(1);


//    // Write depth map as a .bmp image
//    imageWriter->SetFileName("depthmap.bmp");
//    imageWriter->SetInputConnection(scale->GetOutputPort());
//    imageWriter->Write();

  return 0;
}
