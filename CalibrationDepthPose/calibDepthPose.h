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

#ifndef CALIBDEPTHPOSE_H
#define CALIBDEPTHPOSE_H

#include <Eigen/Dense>

#include "matchingMatrix.h"
#include "pclUtils.h"

class CalibParameters;

namespace CalibrationDepthPose
{

struct CalibrationRaw
{
  double qw, qx, qy, qz, x, y, z;

  const Eigen::Isometry3d& toIsometry3d() const {
    return Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);
  }

  void fromIsometry3d(Eigen::Isometry3d const& h) {
    Eigen::Quaterniond q(h.rotation());
    qw = q.w(); qx = q.x(); qy = q.y(); qz = q.z();
    x = h.translation().x();
    y = h.translation().y();
    z = h.translation().z();
  }

  double* data() {
    return reinterpret_cast<double*>(this);
  }
};

std::ostream& operator <<(std::ostream& os, CalibrationRaw const& calib);



class CalibDepthPose
{
public:
  CalibDepthPose(std::vector<Pointcloud::Ptr> const& pointclouds, std::vector<Eigen::Isometry3d> const& poses, Eigen::Isometry3d const& initial_calib);
  Eigen::Isometry3d calibrate(CalibParameters *params);

  MatchingMatrix& getMatchingMatrix() {
    return m_matchMatrix;
  }

private:
  std::vector<Pointcloud::Ptr> m_pointclouds;     // list of pointclouds
  std::vector<Eigen::Isometry3d> m_poses;         // list of poses
  MatchingMatrix m_matchMatrix;
  CalibrationRaw m_calib;
};

}
#endif // CALIBDEPTHPOSE_H
