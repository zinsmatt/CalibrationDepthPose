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

#include <random>

#include <unsupported/Eigen/EulerAngles>

#include "exampleUtils.h"


std::ostream &operator <<(std::ostream &os, const Eigen::Isometry3d &pose)
{
  Eigen::Quaterniond q(pose.rotation());
  os << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "
     << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z();
  return os;
}


void addNoise(CalibrationDepthPose::Pointcloud::Ptr pc, double stddev)
{
  std::default_random_engine generator;
  std::normal_distribution<double> distrib(0.0, stddev);

  for (size_t i = 0; i < pc->size(); ++i)
  {
    double noise = distrib(generator);
    pc->points[i].x *= (1.0 + noise);
    pc->points[i].y *= (1.0 + noise);
    pc->points[i].z *= (1.0 + noise);
  }
}

std::istream &operator >>(std::istream &is, Eigen::Isometry3d &pose)
{
  double qw, qx, qy, qz, x, y, z;
  is >> qw >> qx >> qy >> qz >> x >> y >> z;
  pose = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);
  return is;
}


ColoredPointcloud::Ptr colorizePointCloud(CalibrationDepthPose::Pointcloud::Ptr pc,
                                          unsigned char r,
                                          unsigned char b,
                                          unsigned char g)
{
  ColoredPointcloud::Ptr tempCloud(new ColoredPointcloud);
  tempCloud->resize(pc->size());
  for (size_t i = 0; i < pc->size(); ++i)
  {
    tempCloud->points[i].x = pc->points[i].x;
    tempCloud->points[i].y = pc->points[i].y;
    tempCloud->points[i].z = pc->points[i].z;
    tempCloud->points[i].r = r;
    tempCloud->points[i].g = g;
    tempCloud->points[i].b = b;
  }
  return tempCloud;
}


Eigen::Isometry3d extractIsometry(const YAML::Node &node)
{
  auto rot = node["rotation"].as<std::vector<double>>();
  Eigen::EulerAnglesXYZd r(TO_RADIANS(rot[0]), TO_RADIANS(rot[1]), TO_RADIANS(rot[2]));
  auto transl = node["translation"].as<std::vector<double>>();
  return Eigen::Translation3d(transl[0], transl[1], transl[2]) * r;
}

std::pair<Eigen::Isometry3d, Eigen::Isometry3d> loadConfiguration(const std::string &filename)
{
  YAML::Node config = YAML::LoadFile(filename);
  auto real_calib_node = config["real_calibration"];
  auto estim_calib_node = config["estimated_calibration"];

  Eigen::Isometry3d real_calib = extractIsometry(real_calib_node);
  Eigen::Isometry3d estim_calib = extractIsometry(estim_calib_node);

  return std::make_pair(real_calib, estim_calib);
}

std::string printEulerAngleIsometry(const Eigen::Isometry3d &pose)
{
  Eigen::EulerAnglesXYZd rot(pose.rotation());
  std::stringstream ss;
  ss << TO_DEGREES(rot.alpha()) << " " << TO_DEGREES(rot.beta())<< " " << TO_DEGREES(rot.gamma()) << " "
     << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z();
  return ss.str();
}
