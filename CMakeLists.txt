cmake_minimum_required(VERSION 3.25)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(options)
include(fetch_project_options)

run_vcpkg()
project(ORB_SLAM3)

include(custom_project_options)

execute_process(
  COMMAND ${CMAKE_COMMAND} -E tar -x ORBvoc.txt.tar.gz
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED)
target_sources(${PROJECT_NAME}
  PRIVATE
  src/Atlas.cc
  src/CameraModels/KannalaBrandt8.cpp
  src/CameraModels/Pinhole.cpp
  src/Config.cc
  src/Converter.cc
  src/Frame.cc
  src/FrameDrawer.cc
  src/G2oTypes.cc
  src/GeometricTools.cc
  src/ImuTypes.cc
  src/KeyFrame.cc
  src/KeyFrameDatabase.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/MLPnPsolver.cpp
  src/Map.cc
  src/MapDrawer.cc
  src/MapPoint.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/OptimizableTypes.cpp
  src/Optimizer.cc
  src/Settings.cc
  src/Sim3Solver.cc
  src/System.cc
  src/Tracking.cc
  src/TwoViewReconstruction.cc
  src/Viewer.cc
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels
)

target_compile_definitions(${PROJECT_NAME}
  PUBLIC
  -DCOMPILEDWITHC11
)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
  CXX_EXTENSIONS ON
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

# TODO: Use target_find_dependencies complex mode once the new version of aminya/project_options is released
find_package(Boost CONFIG REQUIRED COMPONENTS serialization)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV 4.4 CONFIG REQUIRED)
find_package(OpenSSL CONFIG REQUIRED)
find_package(Pangolin CONFIG REQUIRED)
if(APPLE AND TARGET pango_opengl)
  find_library(OPENGL_FRAMEWORK OpenGL REQUIRED)
  get_target_property(PANGOLIN_OPENGL_LIBRARIES
    pango_opengl INTERFACE_LINK_LIBRARIES)
  list(FILTER PANGOLIN_OPENGL_LIBRARIES EXCLUDE REGEX
    "^(/usr/X11R6|/opt/X11)/lib/libGL(U)?\\.dylib$")
  list(APPEND PANGOLIN_OPENGL_LIBRARIES "${OPENGL_FRAMEWORK}")
  set_property(TARGET pango_opengl PROPERTY INTERFACE_LINK_LIBRARIES
    "${PANGOLIN_OPENGL_LIBRARIES}")
endif()
find_package(realsense2)

add_subdirectory(Thirdparty)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES}
  Boost::serialization
  DBoW2
  Eigen3::Eigen
  OpenSSL::Crypto
  g2o
  sophus
)

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
  target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${realsense_INCLUDE_DIR}
  )
  target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${realsense2_LIBRARY}
  )
endif()

# Build examples

# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
  Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

if(realsense2_FOUND)
  add_executable(rgbd_realsense_D435i
    Examples/RGB-D/rgbd_realsense_D435i.cc)
  target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})
endif()

# RGB-D inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)

if(realsense2_FOUND)
  add_executable(rgbd_inertial_realsense_D435i
    Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
  target_link_libraries(rgbd_inertial_realsense_D435i ${PROJECT_NAME})
endif()

# Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
  Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
  Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})

add_executable(stereo_tum_vi
  Examples/Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
  add_executable(stereo_realsense_t265
    Examples/Stereo/stereo_realsense_t265.cc)
  target_link_libraries(stereo_realsense_t265 ${PROJECT_NAME})

  add_executable(stereo_realsense_D435i
    Examples/Stereo/stereo_realsense_D435i.cc)
  target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})
endif()

# Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_webcam
  Examples/Monocular/mono_webcam.cc
  Examples/Monocular/DenseMapper.cc)
target_link_libraries(mono_webcam ${PROJECT_NAME})

add_executable(mono_kitti
  Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
  Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

add_executable(mono_tum_vi
  Examples/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
  add_executable(mono_realsense_t265
    Examples/Monocular/mono_realsense_t265.cc)
  target_link_libraries(mono_realsense_t265 ${PROJECT_NAME})

  add_executable(mono_realsense_D435i
    Examples/Monocular/mono_realsense_D435i.cc)
  target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})
endif()

# Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular-Inertial)

add_executable(mono_inertial_euroc
  Examples/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi
  Examples/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
  add_executable(mono_inertial_realsense_t265
    Examples/Monocular-Inertial/mono_inertial_realsense_t265.cc)
  target_link_libraries(mono_inertial_realsense_t265 ${PROJECT_NAME})

  add_executable(mono_inertial_realsense_D435i
    Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
  target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})
endif()

# Stereo Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo-Inertial)

add_executable(stereo_inertial_euroc
  Examples/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc ${PROJECT_NAME})

add_executable(stereo_inertial_tum_vi
  Examples/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi ${PROJECT_NAME})

if(realsense2_FOUND)
  add_executable(stereo_inertial_realsense_t265
    Examples/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
  target_link_libraries(stereo_inertial_realsense_t265 ${PROJECT_NAME})

  add_executable(stereo_inertial_realsense_D435i
    Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
  target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Calibration)

if(realsense2_FOUND)
  add_executable(recorder_realsense_D435i
    Examples/Calibration/recorder_realsense_D435i.cc)
  target_link_libraries(recorder_realsense_D435i ${PROJECT_NAME})

  add_executable(recorder_realsense_T265
    Examples/Calibration/recorder_realsense_T265.cc)
  target_link_libraries(recorder_realsense_T265 ${PROJECT_NAME})
endif()

# Old examples

# RGB-D examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D)

add_executable(rgbd_tum_old
  Examples_old/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum_old ${PROJECT_NAME})

if(realsense2_FOUND)
  add_executable(rgbd_realsense_D435i_old
    Examples_old/RGB-D/rgbd_realsense_D435i.cc)
  target_link_libraries(rgbd_realsense_D435i_old ${PROJECT_NAME})
endif()

# RGB-D inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/RGB-D-Inertial)

if(realsense2_FOUND)
  add_executable(rgbd_inertial_realsense_D435i_old
    Examples_old/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
  target_link_libraries(rgbd_inertial_realsense_D435i_old ${PROJECT_NAME})
endif()

# Stereo examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo)

add_executable(stereo_kitti_old
  Examples_old/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti_old ${PROJECT_NAME})

add_executable(stereo_euroc_old
  Examples_old/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc_old ${PROJECT_NAME})

add_executable(stereo_tum_vi_old
  Examples_old/Stereo/stereo_tum_vi.cc)
target_link_libraries(stereo_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
  add_executable(stereo_realsense_t265_old
    Examples_old/Stereo/stereo_realsense_t265.cc)
  target_link_libraries(stereo_realsense_t265_old ${PROJECT_NAME})

  add_executable(stereo_realsense_D435i_old
    Examples_old/Stereo/stereo_realsense_D435i.cc)
  target_link_libraries(stereo_realsense_D435i_old ${PROJECT_NAME})
endif()

# Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular)

add_executable(mono_tum_old
  Examples_old/Monocular/mono_tum.cc)
target_link_libraries(mono_tum_old ${PROJECT_NAME})

add_executable(mono_kitti_old
  Examples_old/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti_old ${PROJECT_NAME})

add_executable(mono_euroc_old
  Examples_old/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc_old ${PROJECT_NAME})

add_executable(mono_tum_vi_old
  Examples_old/Monocular/mono_tum_vi.cc)
target_link_libraries(mono_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
  add_executable(mono_realsense_t265_old
    Examples_old/Monocular/mono_realsense_t265.cc)
  target_link_libraries(mono_realsense_t265_old ${PROJECT_NAME})

  add_executable(mono_realsense_D435i_old
    Examples_old/Monocular/mono_realsense_D435i.cc)
  target_link_libraries(mono_realsense_D435i_old ${PROJECT_NAME})
endif()

# Monocular inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Monocular-Inertial)

add_executable(mono_inertial_euroc_old
  Examples_old/Monocular-Inertial/mono_inertial_euroc.cc)
target_link_libraries(mono_inertial_euroc_old ${PROJECT_NAME})

add_executable(mono_inertial_tum_vi_old
  Examples_old/Monocular-Inertial/mono_inertial_tum_vi.cc)
target_link_libraries(mono_inertial_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
  add_executable(mono_inertial_realsense_t265_old
    Examples_old/Monocular-Inertial/mono_inertial_realsense_t265.cc)
  target_link_libraries(mono_inertial_realsense_t265_old ${PROJECT_NAME})

  add_executable(mono_inertial_realsense_D435i_old
    Examples_old/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
  target_link_libraries(mono_inertial_realsense_D435i_old ${PROJECT_NAME})
endif()

# Stereo Inertial examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples_old/Stereo-Inertial)

add_executable(stereo_inertial_euroc_old
  Examples_old/Stereo-Inertial/stereo_inertial_euroc.cc)
target_link_libraries(stereo_inertial_euroc_old ${PROJECT_NAME})

add_executable(stereo_inertial_tum_vi_old
  Examples_old/Stereo-Inertial/stereo_inertial_tum_vi.cc)
target_link_libraries(stereo_inertial_tum_vi_old ${PROJECT_NAME})

if(realsense2_FOUND)
  add_executable(stereo_inertial_realsense_t265_old
    Examples_old/Stereo-Inertial/stereo_inertial_realsense_t265.cc)
  target_link_libraries(stereo_inertial_realsense_t265_old ${PROJECT_NAME})

  add_executable(stereo_inertial_realsense_D435i_old
    Examples_old/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
  target_link_libraries(stereo_inertial_realsense_D435i_old ${PROJECT_NAME})
endif()
