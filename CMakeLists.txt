cmake_minimum_required(VERSION 3.16)

project(direct_visual_lidar_calibration CXX)

find_package(fmt CONFIG REQUIRED)
find_package(GTSAM CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL CONFIG REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Ceres CONFIG REQUIRED)

add_library(direct_visual_lidar_calibration
  src/camera/create_camera.cpp
  src/vlcal/common/frame.cpp
  src/vlcal/common/frame_cpu.cpp
  src/vlcal/common/ivox.cpp
  src/vlcal/common/cloud_covariance_estimation.cpp
  src/vlcal/common/time_keeper.cpp
  src/vlcal/common/estimate_fov.cpp
  src/vlcal/common/estimate_pose.cpp
  src/vlcal/common/visual_lidar_data.cpp
#   src/vlcal/preprocess/preprocess.cpp
#   src/vlcal/preprocess/generate_lidar_image.cpp
#   src/vlcal/preprocess/static_point_cloud_integrator.cpp
#   src/vlcal/preprocess/dynamic_point_cloud_integrator.cpp
  src/vlcal/calib/cost_calculator_nid.cpp
  src/vlcal/calib/view_culling.cpp
  src/vlcal/calib/visual_camera_calibration.cpp
)
target_compile_definitions(direct_visual_lidar_calibration PUBLIC -D_USE_MATH_DEFINES)
target_include_directories(direct_visual_lidar_calibration PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIRS}
)
target_link_libraries(direct_visual_lidar_calibration
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
  Ceres::ceres
  fmt::fmt
  gtsam
)
