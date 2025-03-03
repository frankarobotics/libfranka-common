cmake_minimum_required(VERSION 3.0.2)

add_library(libfranka-common INTERFACE)
target_include_directories(libfranka-common INTERFACE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
