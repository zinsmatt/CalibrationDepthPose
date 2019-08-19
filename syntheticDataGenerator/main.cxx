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

#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkMath.h>
#include <vtkOBJReader.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPolyDataMapper.h>
#include <vtkPLYReader.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include "depthMapRenderer.h"
#include "keyPressInteractorStyle.h"

namespace fs = std::filesystem;


int main(int argc, char* argv[])
{
  if (argc < 3)
  {
    std::cerr << "Usage:\n\tSyntheticDataGenerator mesh_file out_dir" << std::endl;
    return -1;
  }
  else
  {
    std::cout << "Press [Space] to take a screenshot" << std::endl;
  }
  const std::string str1(argv[1]);
  const std::string str2(argv[2]);
  fs::path mesh_filename(str1);
  fs::path output_directory(str2);
  if (!fs::exists(mesh_filename))
  {
    std::cerr << "Mesh does not exist" << std::endl;
    return -1;
  }

  if (!fs::exists(output_directory))
  {
    std::cerr << "Output directory does not exist" << std::endl;
    return -1;
  }
  output_directory = fs::absolute(output_directory);

  vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
  camera->SetPosition(0, -4, 0);
  camera->SetFocalPoint(0, 0, 0);
  camera->SetViewUp(0, 0, 1);
  camera->SetViewAngle(60);
  double angle = vtkMath::RadiansFromDegrees(camera->GetViewAngle());

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkAbstractPolyDataReader> fileReader;

  if (mesh_filename.extension() == ".obj")
  {
    fileReader = vtkSmartPointer<vtkOBJReader>::New();
  }
  else if (mesh_filename.extension() == ".ply")
  {
    fileReader = vtkSmartPointer<vtkPLYReader>::New();
  }
  else
  {
    throw std::runtime_error("Mesh format not supported: "
                             + mesh_filename.extension().generic_string());
  }

  // Read the mesh file
  fileReader->SetFileName(mesh_filename.generic_string().c_str());

  //Build visualization enviroment
  mapper->SetInputConnection(fileReader->GetOutputPort());
  actor->SetMapper(mapper);
  renderer->AddActor(actor);
  renderer->SetActiveCamera(camera);
  renWin->AddRenderer(renderer);
  interactor->SetRenderWindow(renWin);
  renWin->SetWindowName("Synthetic Depth Map Generator");
  renWin->Render();

  unsigned int width = renderer->GetSize()[0];
  unsigned int height = renderer->GetSize()[1];
  double fx = static_cast<double>(width)  / (2 * std::tan(angle / 2));
  //double fy = static_cast<double>(height) / (2 * std::tan(angle / 2));


  vtkSmartPointer<KeyPressInteractorStyle> style = vtkSmartPointer<KeyPressInteractorStyle>::New();

  DepthMapRenderer dr(mesh_filename.generic_string(), fx, width, height);
  style->setDepthMapRenderer(&dr);
  style->SetOutputDirectory(output_directory);

  vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
  vtkSmartPointer<vtkOrientationMarkerWidget> widget =
      vtkSmartPointer<vtkOrientationMarkerWidget>::New();
  widget->SetOutlineColor(0.9300, 0.5700, 0.1300);
  widget->SetOrientationMarker(axes);
  widget->SetInteractor(interactor) ;
  widget->SetViewport(0.0, 0.0, 0.4, 0.4);
  widget->SetEnabled(1);
  widget->InteractiveOn();

  interactor->SetInteractorStyle(style);
  interactor->Start();

  return 0;
}
