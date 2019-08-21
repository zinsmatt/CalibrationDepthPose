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

#ifndef CALIBPARAMETERS_H
#define CALIBPARAMETERS_H

#include <string>
#include <thread>

namespace CalibrationDepthPose
{

enum class DistanceType
{
  POINT_TO_POINT,
  POINT_TO_PLANE
};

struct CalibParameters
{
public:
  CalibParameters();

public:
  DistanceType distanceType;
  double matchingMaxDistance;
  double matchingRequiredNbNeighbours;
  double matchingPlaneDiscriminatorThreshold;
  unsigned int nbThreads = std::thread::hardware_concurrency();
};

}

#endif // CALIBPARAMETERS_H
