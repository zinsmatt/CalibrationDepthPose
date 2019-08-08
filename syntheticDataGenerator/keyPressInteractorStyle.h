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

#ifndef KEYPRESSINTERACTORSTYLE_H
#define KEYPRESSINTERACTORSTYLE_H

#include <filesystem>

#include <vtkInteractorStyleTrackballCamera.h>

#include "depthMapRenderer.h"


namespace fs = std::filesystem;

// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static KeyPressInteractorStyle* New();
    vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

    KeyPressInteractorStyle() :
      counter(0)
    {}

    ~KeyPressInteractorStyle();

    void SetOutputDirectory(const fs::path& dir) {
      output_dir = dir;
    }

    virtual void OnKeyPress();

    void setDepthMapRenderer(DepthMapRenderer* pt) {
      depthmap_renderer = pt;
    }

private:
    DepthMapRenderer* depthmap_renderer = nullptr;
    int counter = 0;
    fs::path output_dir;
    std::vector<Eigen::Isometry3d> poses;
    std::vector<std::string> pc_filenames;
};


#endif // KEYPRESSINTERACTORSTYLE_H
