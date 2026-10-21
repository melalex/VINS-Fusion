# VINS-Fusion without ROS: the estimator (vins_estimator/src, minus its ROS
# publishers) and the camodocal camera models as plain shared libraries,
# libvins_lib.so and libcamera_models.so, for use as a C++ library.
#
#   cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j8
#
# The ROS packages (vins_estimator/, camera_models/, loop_fusion/, ...) keep
# their own catkin CMakeLists.txt; catkin only builds directories with a
# package.xml, so it ignores this file.
cmake_minimum_required(VERSION 3.16)
project(vins_fusion_noros CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17) # Ceres >= 2.2 requires C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

# camodocal's camera models only - its calibration tool (calib/, chessboard/,
# intrinsic_calib.cc) is not part of the library here.
add_library(camera_models SHARED
  camera_models/src/camera_models/Camera.cc
  camera_models/src/camera_models/CameraFactory.cc
  camera_models/src/camera_models/CostFunctionFactory.cc
  camera_models/src/camera_models/PinholeCamera.cc
  camera_models/src/camera_models/PinholeFullCamera.cc
  camera_models/src/camera_models/CataCamera.cc
  camera_models/src/camera_models/EquidistantCamera.cc
  camera_models/src/camera_models/ScaramuzzaCamera.cc
  camera_models/src/sparse_graph/Transform.cc
  camera_models/src/gpl/gpl.cc
)
target_include_directories(camera_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/camera_models/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(camera_models PUBLIC ${OpenCV_LIBS} Ceres::ceres Eigen3::Eigen Boost::filesystem Boost::system)

# vins_estimator's library sources, without utility/visualization.cpp and
# utility/CameraPoseVisualization.cpp (ROS publishers).
add_library(vins_lib SHARED
  vins_estimator/src/estimator/parameters.cpp
  vins_estimator/src/estimator/estimator.cpp
  vins_estimator/src/estimator/feature_manager.cpp
  vins_estimator/src/factor/pose_local_parameterization.cpp
  vins_estimator/src/factor/projectionTwoFrameOneCamFactor.cpp
  vins_estimator/src/factor/projectionTwoFrameTwoCamFactor.cpp
  vins_estimator/src/factor/projectionOneFrameTwoCamFactor.cpp
  vins_estimator/src/factor/marginalization_factor.cpp
  vins_estimator/src/utility/utility.cpp
  vins_estimator/src/initial/solve_5pts.cpp
  vins_estimator/src/initial/initial_aligment.cpp
  vins_estimator/src/initial/initial_sfm.cpp
  vins_estimator/src/initial/initial_ex_rotation.cpp
  vins_estimator/src/featureTracker/feature_tracker.cpp
)
target_include_directories(vins_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vins_estimator/src>
  $<INSTALL_INTERFACE:include/vins>
)
target_link_libraries(vins_lib PUBLIC camera_models ${OpenCV_LIBS} Ceres::ceres Eigen3::Eigen)

include(GNUInstallDirs)
install(TARGETS camera_models vins_lib LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY camera_models/include/ DESTINATION include)
install(DIRECTORY vins_estimator/src/ DESTINATION include/vins FILES_MATCHING PATTERN "*.h")
