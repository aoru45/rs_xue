#include <iostream>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "realtime_lidar_client.h"
#include "pcap_converter.h"

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(rs_xue, m) {
    m.doc() = "RoboSense LiDAR driver with real-time support"; // 模块文档字符串
    
    // pcap处理函数
    m.def("convert_pcap", &convert_pcap, "read pcd from pcd file");
    m.def("convert_pcap_with_calib", &convert_pcap_with_calib, "read pcd from pcap file and apply calibration and range filtering",
          py::arg("from_name"), py::arg("to_name"), py::arg("R"), py::arg("t"), py::arg("ranges"), py::arg("num_frames"));
    
    // 绑定RealtimeLidarClient类
    py::class_<rs_realtime::RealtimeLidarClient>(m, "Client")
        .def(py::init<>())
        .def("initialize", 
             [](rs_realtime::RealtimeLidarClient& self, const std::string& lidar_ip) {
                 return self.initialize(lidar_ip, 6699, 7788, robosense::lidar::LidarType::RSEM4, "0.0.0.0");
             },
             "Initialize with LiDAR IP (uses default port 6699 and RSEM4 type)",
             py::arg("lidar_ip"))
        .def("get", &rs_realtime::RealtimeLidarClient::get_numpy,
             "Get point cloud data as numpy array with shape (N, 3) containing [x, y, z] coordinates")
        .def("set_calib", &rs_realtime::RealtimeLidarClient::set_calib,
             "Set calibration parameters R (3x3) and t (3x1)")
        .def("stop", &rs_realtime::RealtimeLidarClient::stop,
             "Stop the LiDAR client");
}

