cmake_minimum_required(VERSION 3.22)

# Standalone build of the ekf_rio / reve estimation core, with every ROS,
# catkin and PCL dependency replaced by the headers in shim/. Deliberately
# excluded: all *_ros.cpp, all nodes/*, rio_utils/src/{simple_profiler,
# radar_point_cloud}.cpp, reve's {radar_point_cloud,simple_profiler,
# radar_body_velocity_estimator*}.cpp, and the ekf_yrio / x_rio / gnss_x_rio
# packages.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(pyekf_rio VERSION 0.1.0 LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)

  option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
  option(PYEKF_RIO_WITH_ODR "Build reve's ODR refinement (needs a Fortran compiler)" ON)
endif()

# Fork root: <this file>/../.. == rio/
set(RIO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(REVE "${RIO_ROOT}/reve/radar_ego_velocity_estimator")

if(NOT EXISTS "${REVE}/src/radar_ego_velocity_estimator.cpp")
  message(FATAL_ERROR
    "reve submodule missing. Run: git -C ${RIO_ROOT} submodule update --init reve")
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(pyekf_rio_core
  ${RIO_ROOT}/rio_utils/src/strapdown.cpp
  ${RIO_ROOT}/ekf_rio/src/ekf_rio_filter.cpp
  ${REVE}/src/radar_ego_velocity_estimator.cpp
  shim_impl/radar_point_cloud_shim.cpp
  pyekf_rio/config.cpp
  pyekf_rio/ekf_rio_runner.cpp
)

target_include_directories(pyekf_rio_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}          # pyekf_rio/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/shim     # ros/, angles/, pcl/, tf2/, *Config.h
  ${RIO_ROOT}/rio_utils/include
  ${RIO_ROOT}/ekf_rio/include
  ${REVE}/include
)

# Upstream spells the include both ways: rio_utils/math_helper.h uses
# <Eigen/Dense> while the data_types.h headers use <eigen3/Eigen/Dense>.
# Eigen3::Eigen only supplies the former, so add the parent directory too.
target_include_directories(pyekf_rio_core SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}/..
)

target_link_libraries(pyekf_rio_core PUBLIC Eigen3::Eigen yaml-cpp Boost::headers)
# Upstream's release flags add -march=native; leave it off so the extension
# stays portable across the cluster's node types.
target_compile_options(pyekf_rio_core PRIVATE -O3)

if(PYEKF_RIO_WITH_ODR)
  enable_language(Fortran)

  # A conda-forge gfortran alongside a system C++ compiler drags conda's
  # sysroot onto the link line, where its libm.so linker script points at a
  # /lib64/libm.so.6 that does not exist outside the sysroot. Drop those
  # directories; the Fortran runtime itself still resolves from $CONDA_PREFIX/lib.
  set(_fortran_link_dirs "")
  foreach(_dir IN LISTS CMAKE_Fortran_IMPLICIT_LINK_DIRECTORIES)
    if(NOT _dir MATCHES "sysroot")
      list(APPEND _fortran_link_dirs "${_dir}")
    endif()
  endforeach()
  set(CMAKE_Fortran_IMPLICIT_LINK_DIRECTORIES "${_fortran_link_dirs}")

  add_library(odrpack
    ${REVE}/src/odrpack/d_odr.f
    ${REVE}/src/odrpack/d_mprec.f
    ${REVE}/src/odrpack/lpkbls.f
    ${REVE}/src/odrpack/dlunoc.f
    ${REVE}/src/odrpack/real_precision.f
  )
  set_property(TARGET odrpack PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_sources(pyekf_rio_core PRIVATE ${REVE}/src/odr.cpp)
  target_link_libraries(pyekf_rio_core PRIVATE odrpack)
else()
  target_sources(pyekf_rio_core PRIVATE shim_impl/odr_stub.cpp)
endif()

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR AND BUILD_PYTHON_BINDINGS)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../python
                   ${CMAKE_CURRENT_BINARY_DIR}/python)
endif()
