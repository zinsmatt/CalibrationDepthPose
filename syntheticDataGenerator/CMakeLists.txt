
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
find_package(PCL REQUIRED)
find_package(kwiver REQUIRED)
add_definitions(${PCL_DEFINITIONS})


set(targetName "SyntheticDataGenerator")

include_directories( ${KWIVER_INCLUDE_DIRS} )
link_directories( ${KWIVER_LIBRARY_DIR} )

link_directories("/home/matthieu/opt/kwiver/lib/")

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
