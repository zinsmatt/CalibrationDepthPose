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
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdlib.h>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <yaml-cpp/yaml.h>

#include <CalibrationDepthPose/calibDepthPose.h>
#include <CalibrationDepthPose/calibParameters.h>
#include <CalibrationDepthPose/eigenUtils.h>

#include "exampleUtils.h"

namespace fs = std::filesystem;
using namespace CalibrationDepthPose;


int main(int argc, char* argv[])
{
  srand (time(NULL));
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  if (argc < 5)
  {
    std::cerr << "Usage:\n\t calibrate_synthetic_data dataset_file nb_iterations "
                 "noise_stddev calibrations_files\n" << std::endl;
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
  CalibrationDepthPose::Isometry3d_vector poses;
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
  params.nbThreads = std::thread::hardware_concurrency() * 2 + 1;

  int nbIterations = 20;
  try {
    nbIterations = std::stoi(argv[2]);
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    std::cerr << "Default number of iterations " << nbIterations << std::endl;
  }

  // Load the real calibration and the initial guess
  YAML::Node config = YAML::LoadFile(argv[4]);
  Eigen::Isometry3d realCalib = extractIsometry(config["real_calibration"]);
  Eigen::Isometry3d estimatedCalib = extractIsometry(config["estimated_calibration"]);

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

  std::cout << "\nWith Euler angles:\n";
  std::cout << "Real calibration: " << printEulerAngleIsometry(realCalib) << "\n";
  std::cout << "Estimated calibration: " << printEulerAngleIsometry(estimatedCalib) << std::endl;


  return 0;
}
