
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
find_package(PCL REQUIRED)
find_package(kwiver REQUIRED)
add_definitions(${PCL_DEFINITIONS})


include_directories( ${KWIVER_INCLUDE_DIRS} )
link_directories( ${KWIVER_LIBRARY_DIR} )

link_directories("/home/matthieu/opt/kwiver/lib/")

add_executable(CalibDepthPoseExample
    main.cxx
    depthMapRenderer.cxx
    depthMapRenderer.h
    )

target_include_directories(CalibDepthPoseExample SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
    )

target_link_libraries(CalibDepthPoseExample PRIVATE
    CalibDepthPose
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    ${KWIVER_LIBRARIES}
    stdc++fs
    )
