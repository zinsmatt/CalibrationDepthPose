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
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <stdlib.h>
#include <time.h>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <yaml-cpp/yaml.h>

#include "calibDepthPose.h"
#include "calibParameters.h"

namespace fs = std::filesystem;
using namespace CalibrationDepthPose;

using ColoredPoint = pcl::PointXYZRGB;
using ColoredPointcloud = pcl::PointCloud<ColoredPoint>;


std::ostream& operator <<(std::ostream& os, Eigen::Isometry3d const& pose)
{
  Eigen::Quaterniond q(pose.rotation());
  os << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "
     << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z();
  return os;
}

/// Add Gaussian noise to point cloud
void addNoise(Pointcloud::Ptr pc, double stddev)
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


std::istream& operator >>(std::istream &is, Eigen::Isometry3d& pose)
{
  double qw, qx, qy, qz, x, y, z;
  is >> qw >> qx >> qy >> qz >> x >> y >> z;
  pose = Eigen::Translation3d(x, y, z) * Eigen::Quaterniond(qw, qx, qy, qz);
  return is;
}


/// Colorize the point cloud with rgb
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

/// Extract an isometry transform from a YAML node
Eigen::Isometry3d extractIsometry(YAML::Node const& node)
{
  auto rot = node["rotation"].as<std::vector<double>>();
  Eigen::Quaterniond q(rot[0], rot[1], rot[2], rot[3]);
  auto transl = node["translation"].as<std::vector<double>>();
  return Eigen::Translation3d(transl[0], transl[1], transl[2]) * q;
}

/// Load configuration from YAML file
std::pair<Eigen::Isometry3d, Eigen::Isometry3d> loadConfiguration(std::string const& filename)
{
  YAML::Node config = YAML::LoadFile(filename);
  auto real_calib_node = config["real_calibration"];
  auto estim_calib_node = config["estimated_calibration"];

  Eigen::Isometry3d real_calib = extractIsometry(real_calib_node);
  Eigen::Isometry3d estim_calib = extractIsometry(estim_calib_node);

  return std::make_pair(real_calib, estim_calib);
}


int main(int argc, char* argv[])
{
  srand (time(NULL));
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  if (argc < 5)
  {
    std::cerr << "Usage:\n\tCalibDepthPoseExample dataset_file nb_iterations noise_stddev configuration_file\n" << std::endl;
    return -1;
  }

  // Load dataset
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

  // Load pointclouds
  std::vector<CalibrationDepthPose::Pointcloud::Ptr> pointclouds;
  std::vector<Eigen::Isometry3d> poses;
  // set the noise level
  double noise_stddev = 0.0;
  try {
    noise_stddev = std::stod(argv[3]);
  } catch(std::exception& e) {
    std::cerr << e.what() << std::endl;
    std::cerr << "Default noise " << noise_stddev << std::endl;
  }

  for (auto const& s : pc_files)
  {
    CalibrationDepthPose::Pointcloud::Ptr pc(new CalibrationDepthPose::Pointcloud());
    if (pcl::io::loadPLYFile(s, *pc) != 0)
    {
      throw std::runtime_error("Could not load point cloud: " + s);
    }
    addNoise(pc, noise_stddev);
    pointclouds.emplace_back(pc);
  }

  // Load poses
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

  int nbIterations = 20;
  try {
    nbIterations = std::stoi(argv[2]);
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    std::cerr << "Default number of iterations " << nbIterations << std::endl;
  }


  auto [realCalib, estimatedCalib] = loadConfiguration(argv[4]);
  std::cout << "\nReal calibration: " << realCalib << "\n";
  std::cout << "Initial guess of calibration: " << estimatedCalib << std::endl;

  // Update the poses to have the real calib between the camera and the poses
  Eigen::Isometry3d realCalib_inv = realCalib.inverse();
  for (auto& p : poses)
  {
    p = p * realCalib_inv;
  }

  std::cout << "\nStart calibration" << std::endl;
  // Estimation of the calib
//  Eigen::Isometry3d estimatedCalib = Eigen::Isometry3d::Identity();
  CalibDepthPose calibration(pointclouds, poses, estimatedCalib);
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
      auto transformed_pc = transform(pointclouds[i], poses[i] * estimatedCalib);
      auto colored = colorizePointCloud(transformed_pc, colors[i][0], colors[i][1], colors[i][2]);
      *concat += *colored;
    }
    pcl::io::savePLYFile("colored_" + std::to_string(iter) + ".ply", *concat);

    calibration.calibIteration(&params);
    estimatedCalib = calibration.getCurrentCalibration();
  }


  // Output the retransformed pointclouds with the final calibration estimate
  ColoredPointcloud::Ptr concat(new ColoredPointcloud);
  for (size_t i = 0; i < pointclouds.size(); ++i)
  {
    auto transformed_pc = transform(pointclouds[i], poses[i] * estimatedCalib);
    auto colored = colorizePointCloud(transformed_pc, colors[i][0], colors[i][1], colors[i][2]);
    *concat += *colored;
  }
  pcl::io::savePLYFile("colored_" + std::to_string(nbIterations) + ".ply", *concat);

  std::cout << "\nReal calibration: " << realCalib << "\n";
  std::cout << "Estimated calibration: " << estimatedCalib << std::endl;

  return 0;
}
