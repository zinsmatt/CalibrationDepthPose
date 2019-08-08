#=========================================================================
#
# Copyright 2019 Kitware, Inc.
# Author: Matthieu Zins
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#=========================================================================

cmake_minimum_required(VERSION 3.0)
project(CalibrationDepthPose)

option(BUILD_SYNTHETIC_DATA "Build the synthetic data generator project" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

add_definitions(${PCL_DEFINITIONS})

set(SOURCES_DIR CalibrationDepthPose)
# Headers
set(${PROJECT_NAME}_headers
    ${SOURCES_DIR}/calibDepthPose.h
    ${SOURCES_DIR}/pclUtils.h
    )

# Sources
set(${PROJECT_NAME}_sources
    ${SOURCES_DIR}/calibDepthPose.cxx
    ${SOURCES_DIR}/pclUtils.cxx
    )

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_headers} ${${PROJECT_NAME}_sources})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${SOURCE_DIR}
    )

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    )

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${CERES_LIBRARIES}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    )

if (BUILD_SYNTHETIC_DATA)
    add_subdirectory(syntheticDataGenerator)
endif (BUILD_SYNTHETIC_DATA)


