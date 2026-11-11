# VINS-Mono -- ROS-free C++17 port, EuRoC MAV version (MH_01_easy ...).
# Windows: VS 2022 Build Tools + vcpkg (ceres, opencv4, eigen3) -> scripts/setup_vcpkg.ps1, scripts/build.ps1
# Linux  : apt install libceres-dev libopencv-dev libeigen3-dev; cmake -B build -DCMAKE_BUILD_TYPE=Release; cmake --build build -j
cmake_minimum_required(VERSION 3.20)

# --- Windows: find vcpkg automatically if no toolchain file was given ---------
if(WIN32 AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  # VCPKG_ROOT first, then the private copy made by scripts/setup_vcpkg.ps1,
  # then common install locations.
  set(_vcpkg_candidates "$ENV{VCPKG_ROOT}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/vcpkg"
                        "C:/vcpkg" "C:/src/vcpkg" "C:/tools/vcpkg" "$ENV{USERPROFILE}/vcpkg")
  foreach(_c ${_vcpkg_candidates})
    if(_c AND EXISTS "${_c}/scripts/buildsystems/vcpkg.cmake")
      set(CMAKE_TOOLCHAIN_FILE "${_c}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "vcpkg toolchain")
      message(STATUS "Using vcpkg toolchain: ${CMAKE_TOOLCHAIN_FILE}")
      break()
    endif()
  endforeach()
endif()
if(WIN32 AND NOT DEFINED VCPKG_TARGET_TRIPLET)
  set(VCPKG_TARGET_TRIPLET "x64-windows" CACHE STRING "vcpkg triplet")
endif()

project(vins_mono_euroc LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs video calib3d features2d)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)

file(GLOB CAMERA_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/camera_model/src/*.cc)
file(GLOB TRACKER_SRC CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/feature_tracker/*.cpp)
file(GLOB ESTIMATOR_SRC CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/vins_estimator/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/src/vins_estimator/factor/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/src/vins_estimator/initial/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/src/vins_estimator/utility/*.cpp)

add_library(vins_core STATIC ${CAMERA_SRC} ${TRACKER_SRC} ${ESTIMATOR_SRC} src/common/ros_compat.cpp)
target_include_directories(vins_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common ${CMAKE_CURRENT_SOURCE_DIR}/src/camera_model
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vins_estimator ${CMAKE_CURRENT_SOURCE_DIR}/src/feature_tracker ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vins_core PUBLIC ${OpenCV_LIBS} Eigen3::Eigen Ceres::ceres)

# EuRoC driver (replaces feature_tracker_node + vins_estimator node)
add_executable(vins_euroc src/app/main_euroc.cpp src/app/euroc_dataset.cpp)
target_link_libraries(vins_euroc PRIVATE vins_core)

foreach(t vins_core vins_euroc)
  if(MSVC)
    target_compile_options(${t} PRIVATE /W3 /EHsc /bigobj /utf-8 /permissive- /wd4244 /wd4267 /wd4305 /wd4996 $<$<CONFIG:Release>:/O2 /Oi /fp:precise>)
    target_compile_definitions(${t} PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX _USE_MATH_DEFINES GLOG_NO_ABBREVIATED_SEVERITIES)
  else()
    target_compile_options(${t} PRIVATE -Wall -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable -O2)
  endif()
endforeach()
