cmake_minimum_required(VERSION 3.16)
project(rs_xue VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

if(MSVC)
  add_compile_options(/W3)
else()
  add_compile_options(-Wno-deprecated-declarations)
endif()

set(PYBIND11_FINDPYTHON ON)

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
message(STATUS "Using Python: ${Python3_EXECUTABLE}")

add_subdirectory(pybind11)
pybind11_add_module(rs_xue
    rs_xue/binding.cc
    rs_xue/realtime_lidar_client.cpp
    rs_xue/pcap_converter.cpp
    rs_xue/avi_writer.cpp
    rs_xue/avi_driver.h
)

find_package(ZLIB REQUIRED)
find_package(PcapPlusPlus CONFIG REQUIRED)
find_package(avcpp REQUIRED)
set(rs_driver_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR})
add_library(rs_driver INTERFACE)
target_include_directories(rs_driver INTERFACE ${rs_driver_INCLUDE_DIRS})
target_link_libraries(rs_driver INTERFACE
  PCAP::PCAP
)
if(WIN32)
  target_link_libraries(rs_driver INTERFACE ws2_32)
else()
  target_link_libraries(rs_driver INTERFACE pthread)
endif()

target_include_directories(rs_xue PRIVATE
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  ${rs_driver_INCLUDE_DIRS}
)

target_link_libraries(rs_xue PRIVATE
  ZLIB::ZLIB
  rs_driver
  avcpp::avcpp
)

install(TARGETS rs_xue
  LIBRARY DESTINATION rs_xue
  RUNTIME DESTINATION rs_xue
  ARCHIVE DESTINATION rs_xue
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
