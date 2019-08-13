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

#include "calibDepthPose.h"

#include <fstream>

#include "calibCostFunctions.h"
#include "matchingTools.h"


namespace
{

void saveMatches(std::vector<Eigen::Vector3d> const& pts1,
                 std::vector<Eigen::Vector3d> const& pts2,
                 std::string const& filename)
{
  std::ofstream file(filename);
  for (size_t i = 0; i < pts1.size(); ++i)
  {
    file << "v " << pts1[i].x() << " " << pts1[i].y() << " " << pts1[i].z() << "\n";
    file << "v " << pts2[i].x() << " " << pts2[i].y() << " " << pts2[i].z() << "\n";
    file << "l " << 1 + 2 * i << " " << 1 + 2 * i + 1 << "\n";
  }
  file.close();
}

}
namespace CalibrationDepthPose
{

CalibDepthPose::CalibDepthPose(const std::vector<Pointcloud::Ptr> &pointclouds,
                               const std::vector<Eigen::Isometry3d> &poses,
                               const Eigen::Isometry3d &initial_calib)
  : m_pointclouds(pointclouds), m_poses(poses), m_matchMatrix(MatchingMatrix(pointclouds.size()))
{
  m_calib.fromIsometry3d(initial_calib);
}

Eigen::Isometry3d CalibDepthPose::calibrate(int nbIterations, CalibParameters *params)
{
  for (int i = 0; i < nbIterations; ++i)
  {
    this->calibIteration(params);
  }
  return m_calib.toIsometry3d();
}

void CalibDepthPose::calibIteration(CalibParameters *params)
{
  std::vector<std::vector<Eigen::Vector3d>> keep_pts1, keep_pts2, keep_normals;
  std::vector<Eigen::Isometry3d> keep_posesDiff;
  unsigned int nbTotalMatches = 0;
  for (size_t idx1 = 0; idx1 < m_pointclouds.size(); ++idx1)
  {
    for (size_t idx2 = 0; idx2 < m_pointclouds.size(); ++idx2)
    {
      if (idx1 == idx2 || !m_matchMatrix(idx1, idx2))
        continue;
      auto [pts1, pts2, normals] = matchPointClouds(m_pointclouds[idx1], m_pointclouds[idx2],
          m_poses[idx1] * m_calib.toIsometry3d() , m_poses[idx2] * m_calib.toIsometry3d(), params);
      nbTotalMatches += pts1.size();
//      saveMatches(pts1, pts2, "matches_" + std::to_string(idx1) + "_" + std::to_string(idx2) + ".obj");

      keep_pts1.emplace_back(std::move(pts1));
      keep_pts2.emplace_back(std::move(pts2));
      keep_normals.emplace_back(std::move(normals));
      keep_posesDiff.push_back(m_poses[idx2].inverse() * m_poses[idx1]);
    }
  }



  // Optimization: Non-linear least_square with Ceres
  ceres::Problem problem;
  for (size_t matchIdx = 0; matchIdx < keep_pts1.size(); ++matchIdx)
  {
    for (size_t i = 0; i < keep_pts1[matchIdx].size(); ++i)
    {
      ceres::CostFunction* cost_function = CalibrationResiduals::Create(keep_pts1[matchIdx][i], keep_pts2[matchIdx][i],
                                                                        keep_normals[matchIdx][i], keep_posesDiff[matchIdx]);
      problem.AddResidualBlock(cost_function, nullptr, m_calib.data());
    }
  }

  problem.SetParameterization(m_calib.data(),
                              new ceres::ProductParameterization(
                                new ceres::QuaternionParameterization(),
                                new ceres::IdentityParameterization(3)));

  std::cout << "Total matches = " << nbTotalMatches << std::endl;

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 50;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 8;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;

  std::cout << "Calibration estimation: " << m_calib << std::endl;

}


std::ostream& operator <<(std::ostream& os, CalibrationRaw const& calib)
{
  os << calib.qw << " " << calib.qx << " " << calib.qy << " " << calib.qz << " "
     << calib.x << " " << calib.y << " " << calib.z;
  return os;
}

}
