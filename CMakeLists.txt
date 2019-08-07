cmake_minimum_required(VERSION 3.0)
project(CalibDepthPose)

option(BUILD_EXAMPLE "Build the example project" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(Eigen3 REQUIRED)
#find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

#add_definitions(${PCL_DEFINITIONS})

set(SOURCES_DIR calibDepthPose)


# Headers
set(${PROJECT_NAME}_headers
    ${SOURCES_DIR}/calibDepthPose.h
    )

# Sources
set(${PROJECT_NAME}_sources
    ${SOURCES_DIR}/calibDepthPose.cxx
    )

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_headers} ${${PROJECT_NAME}_sources})

target_include_directories(${PROJECT_NAME} PUBLIC
    ${SOURCE_DIR}
    )

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    #${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    )

target_link_libraries(${PROJECT_NAME} PUBLIC
    ${CERES_LIBRARIES}
    #${PCL_LIBRARIES}
    Eigen3::Eigen
    )

if (BUILD_EXAMPLE)
    add_subdirectory(example)
endif (BUILD_EXAMPLE)


