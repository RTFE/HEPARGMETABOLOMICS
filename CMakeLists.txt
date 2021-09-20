cmake_minimum_required(VERSION 2.8.3)
project(ensenso)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cmake_modules
  cv_bridge
  eigen_conversions
  geometry_msgs
  message_generation
  pcl_conversions
  pcl_ros
  std_msgs
  std_srvs
  tf_conversions
  image_transport
)

find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/opt/ensenso/cmake")
find_package(Ensenso REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(nxLib REQUIRED ensenso-nxlib)
if((nxLib_VERSION VERSION_EQUAL "2.1.7") OR (nxLib_VERSION VERSION_GREATER "2.1.7"))
  add_definitions(-DCUDA_IMPLEMENTED)
  message("CUDA support enabled.")
else()
  message(WARNING "CUDA not supported. Upgrade EnsensoSDK to version >= 2.1.7 in order to use CUDA.")
endif()

catkin_python_setup()


add_message_files(
  FILES
  RawStereoPattern.msg
)

add_service_files(
  FILES
  CalibrateHandEye.srv
 