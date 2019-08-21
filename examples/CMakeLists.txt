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

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
find_package(yaml-cpp REQUIRED)


set(targetName1 "calibrate_synthetic_data")


add_executable(${targetName1}
    main_calibrate_synthetic_data.cxx
    exampleUtils.cxx
    exampleUtils.h
    )

target_include_directories(${targetName1} SYSTEM PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    )

target_link_libraries(${targetName1} PRIVATE
    CalibrationDepthPose
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    stdc++fs
    )


set(targetName2 "calibrate")

add_executable(${targetName2}
    main_calibrate.cxx
    exampleUtils.cxx
    exampleUtils.h
    )

target_include_directories(${targetName2} SYSTEM PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    )

target_link_libraries(${targetName2} PRIVATE
    CalibrationDepthPose
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    stdc++fs
    )
