cmake_minimum_required(VERSION 3.10)
project(pgo_raster_cpp CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
# OpenMP is optional: without it the per-cell loop is simply serial.
find_package(OpenMP QUIET)

add_library(pgo_raster SHARED src/pgo_raster.cpp)
target_include_directories(pgo_raster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(pgo_raster PRIVATE -O3 -fPIC)
if(OpenMP_CXX_FOUND)
  target_link_libraries(pgo_raster PRIVATE OpenMP::OpenMP_CXX)
endif()

install(TARGETS pgo_raster
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

ament_package()
