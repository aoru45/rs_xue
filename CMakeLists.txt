cmake_minimum_required(VERSION 3.16.0)
project(rs_xue VERSION 0.1.0)

add_compile_options(-Wno-deprecated-declarations)
# set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-std=c++11)
set(PYTHON_EXECUTABLE "/home/lab4dv/anaconda3/envs/xue/bin/python")

add_subdirectory(pybind11)
pybind11_add_module(rs_xue rs_xue/binding.cc rs_xue/realtime_lidar_client.cpp rs_xue/pcap_converter.cpp)

add_subdirectory(cnpy)
target_include_directories(rs_xue PRIVATE cnpy)
target_link_libraries(rs_xue PRIVATE cnpy-static z)

find_package(rs_driver REQUIRED)
target_include_directories(rs_xue PRIVATE ${rs_driver_INCLUDE_DIRS})

target_link_libraries(rs_xue PRIVATE ${rs_driver_LIBRARIES})


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
