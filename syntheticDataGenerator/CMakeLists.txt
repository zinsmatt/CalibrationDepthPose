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

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
find_package(PCL REQUIRED)
find_package(kwiver REQUIRED)
add_definitions(${PCL_DEFINITIONS})


set(targetName "SyntheticDataGenerator")

include_directories( ${KWIVER_INCLUDE_DIRS} )
link_directories( ${KWIVER_LIBRARY_DIR} )

add_executable(${targetName}
    main.cxx
    depthMapRenderer.cxx
    depthMapRenderer.h
    keyPressInteractorStyle.cxx
    keyPressInteractorStyle.h
    )

target_include_directories(${targetName} SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
    )

target_link_libraries(${targetName} PRIVATE
    CalibDepthPose
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    ${KWIVER_LIBRARIES}
    stdc++fs
    )
