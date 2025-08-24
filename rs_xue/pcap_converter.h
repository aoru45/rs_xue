#ifndef PCAP_CONVERTER_H
#define PCAP_CONVERTER_H

#include <iostream>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <fstream>
#include <vector>
#include <thread>
#include <iomanip>
#include <sstream>
#include <rs_driver/api/lidar_driver.hpp>
#include "cnpy.h"

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

typedef PointXYZIT PointT;
typedef PointCloudT<PointT> PointCloudMsg;
using namespace robosense::lidar;
namespace py = pybind11;

// 全局队列声明
extern SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
extern SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

// 回调函数声明
std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void);
void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg);
void exceptionCallback(const Error& code);

// 工具函数声明
void saveNpy(const std::string& path, const float* data, const std::vector<size_t>& shape);
void processCloud(const std::string& output_dir, int num_frames);
void processCloudWithCalib(const std::string& output_dir, const float* R, const float* t, const float* ranges, int num_frames);

// 主要转换函数声明
int convert_pcap(const std::string& from_name, const std::string& to_name, int num_frames);
int convert_pcap_with_calib(const std::string& from_name, const std::string& to_name, const py::array_t<float>& R, const py::array_t<float>& t, const py::array_t<float>& ranges, int num_frames);

#endif // PCAP_CONVERTER_H