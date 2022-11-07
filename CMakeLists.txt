cmake_minimum_required(VERSION 3.5)
project(pointcloud_compressor)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(Eigen3 REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(thirdparty/draco)

ament_auto_add_executable(pointcloud_compressor
  src/pointcloud_compressor.cpp
  src/pointcloud_extraction.cpp
  src/pointcloud_downsampling.cpp
  src/pointcloud_draco.cpp
)
target_include_directories(pointcloud_compressor PUBLIC
  include
  thirdparty/draco/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(pointcloud_compressor
  draco
)

ament_auto_add_executable(pointcloud_decompressor
  src/pointcloud_decompressor.cpp
)
target_include_directories(pointcloud_decompressor PUBLIC
  include
  thirdparty/draco/src
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(pointcloud_decompressor
  draco
)

ament_auto_package()