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

#include "keyPressInteractorStyle.h"

#include <iostream>
#include <sstream>

#include <vtkCamera.h>
#include <vtkMath.h>
#include <vtkObjectFactory.h>
#include <vtkRendererCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

vtkStandardNewMacro(KeyPressInteractorStyle);


KeyPressInteractorStyle::~KeyPressInteractorStyle()
{
  const std::string poses_filename = output_dir / "dataset.poses";
  const std::string dataset_filename = output_dir / "dataset.txt";

  std::ofstream file(dataset_filename);
  file << poses_filename << "\n";
  for (int i = 0; i < counter; ++i)
  {
    file << pc_filenames[i] << "\n";
  }
  file.close();

  std::ofstream poses_file(poses_filename);
  for (int i = 0; i < counter; ++i)
  {
    Eigen::Quaterniond q(poses[i].rotation());
    const Eigen::Vector3d& pos(poses[i].translation());
    poses_file << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "
               << pos.x() << " " << pos.y() << " " << pos.z() << "\n";
  }
  poses_file.close();
}


void KeyPressInteractorStyle::OnKeyPress()
{
  // Get the keypress
  vtkRenderWindowInteractor *rwi = this->Interactor;
  std::string key = rwi->GetKeySym();

  if (key == "Escape")
  {
    std::cout << "Quit" << std::endl;
    rwi->TerminateApp();
  }

  if (key != "space")
    return;

  double *q = rwi->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetOrientationWXYZ();
  Eigen::AngleAxisd rotation(vtkMath::RadiansFromDegrees(q[0]), Eigen::Vector3d(q[1], q[2], q[3]));
  double x = 0.0, y = 0.0, z = 0.0;
  rwi->GetRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->GetPosition(x, y, z);

  Eigen::Vector3d cam_position(x, y, z);
  // Additional rotation because the vtk cameras seems to look in -z
  Eigen::Quaterniond cam_rotation = Eigen::Quaterniond(0, 1, 0, 0) * Eigen::Quaterniond(rotation.toRotationMatrix());
  Eigen::Quaterniond cam_orientation = cam_rotation.inverse();
  if (depthmap_renderer)
  {
    depthmap_renderer->setCameraPosition(cam_position);
    depthmap_renderer->setCameraOrientation(cam_rotation);
    auto pc = depthmap_renderer->capturePointCloud();
    std::stringstream ss;
    ss << "pc_" << std::setw(4) << std::setfill('0') << counter++;
    std::cout << "Capture point cloud " << ss.str() << std::endl;
    std::string pc_filename = (output_dir / (ss.str() + ".ply")).generic_string();
    pcl::io::savePLYFile(pc_filename, *pc);
    pc_filenames.push_back(pc_filename);
    poses.push_back(Eigen::Translation3d(cam_position) * cam_orientation);
  }

  // Forward events
  vtkInteractorStyleTrackballCamera::OnKeyPress();
}
