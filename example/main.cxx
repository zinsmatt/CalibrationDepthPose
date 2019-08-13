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
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdlib.h>
#include <time.h>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include "calibDepthPose.h"
#include "calibParameters.h"

namespace fs = std::filesystem;
using namespace CalibrationDepthPose;

using ColoredPoint = pcl::PointXYZRGB;
using ColoredPointcloud = pcl::PointCloud<ColoredPoint>;

std::istream& operator >>(std::istream &is, Eigen::Isometry3d& pose)
{
  double qw, qx, qy, qz, x, y, z;
  is >> qw >> qx >> qy >> qz >> x >> y >> z;
  pose = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);
  return is;
}

ColoredPointcloud::Ptr colorizePointCloud(Pointcloud::Ptr pc,
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



int main(int argc, char* argv[])
{
  srand (time(NULL));
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  if (argc < 2)
  {
    std::cerr << "Usage:\n\tCalibDepthPoseExample dataset_file\n" << std::endl;
    return -1;
  }

  std::string filename(argv[1]);
  std::ifstream dataset_file(filename);
  if (!dataset_file.is_open())
  {
    throw std::runtime_error("Could not read the dataset file: " + filename);
  }
  std::string pose_filename;
  dataset_file >> pose_filename;
  std::vector<std::string> pc_files;
  std::string line;
  while (dataset_file >> line)
  {
    pc_files.push_back(line);
  }
  dataset_file.close();

  std::vector<CalibrationDepthPose::Pointcloud::Ptr> pointclouds;
  std::vector<Eigen::Isometry3d> poses;
  for (auto const& s : pc_files)
  {
    CalibrationDepthPose::Pointcloud::Ptr pc(new CalibrationDepthPose::Pointcloud());
    if (pcl::io::loadPLYFile(s, *pc) != 0)
    {
      throw std::runtime_error("Could not load point cloud: " + s);
    }
    pointclouds.emplace_back(pc);
  }

  std::ifstream pose_file(pose_filename);
  Eigen::Isometry3d pose;
  while (pose_file >> pose)
  {
    poses.emplace_back(pose);
  }
  pose_file.close();

  if (pointclouds.size() != poses.size())
  {
    throw std::runtime_error("Not the same number of pointclouds ("
                             + std::to_string(pointclouds.size())
                             + ") and poses ("
                             + std::to_string(poses.size()) + ")");
  }

  // Generate random colors for outputs (one for each point cloud)
  std::vector<Eigen::Vector3i> colors;
  for (size_t i = 0; i < pointclouds.size(); ++i)
  {
    colors.emplace_back(rand() % 255, rand() % 255, rand() % 255);
  }

  CalibParameters params;
  params.distanceType = DistanceType::POINT_TO_PLANE;
  params.matchingMaxDistance = 0.1;
  params.matchingPlaneDiscriminatorThreshold = 0.8;
  params.matchingRequiredNbNeighbours = 10;
  int nbIterations = 8;

  Eigen::Isometry3d calib = Eigen::Translation3d(0.3, -0.4, -0.1)
      * Eigen::Quaterniond(0.9951613, 0.0419272, -0.0211705, 0.0863012);

  std::cout << "Start calibration" << std::endl;
  CalibDepthPose calibration(pointclouds, poses, calib);
  // Set the pairs of pointclouds used for matching
  for (size_t i = 0; i < pointclouds.size(); ++i)
  {
    calibration.getMatchingMatrix().addMatch(i, (i + 1) % pointclouds.size(), true);
  }

  // Calibration loop
  for (int iter = 0; iter < nbIterations; ++iter)
  {
    // Ouput the retransformed pointclouds with the current calibration estimate
    ColoredPointcloud::Ptr concat(new ColoredPointcloud);
    for (size_t i = 0; i < pointclouds.size(); ++i)
    {
      auto transformed_pc = transform(pointclouds[i], poses[i] * calib);
      auto colored = colorizePointCloud(transformed_pc, colors[i][0], colors[i][1], colors[i][2]);
      *concat += *colored;
    }
    pcl::io::savePLYFile("colored_" + std::to_string(iter) + ".ply", *concat);

    calibration.calibIteration(&params);
    calib = calibration.getCurrentCalibration();
  }


  // Output the retransformed pointclouds with the final calibration estimate
  ColoredPointcloud::Ptr concat(new ColoredPointcloud);
  for (size_t i = 0; i < pointclouds.size(); ++i)
  {
    auto transformed_pc = transform(pointclouds[i], poses[i] * calib);
    auto colored = colorizePointCloud(transformed_pc, colors[i][0], colors[i][1], colors[i][2]);
    *concat += *colored;
  }
  pcl::io::savePLYFile("colored_" + std::to_string(nbIterations) + ".ply", *concat);

  return 0;
}
