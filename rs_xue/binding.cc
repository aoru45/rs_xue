#include <iostream>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "realtime_lidar_client.h"
#include "pcap_converter.h"

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_rs_xue, m) {
    m.doc() = "RoboSense LiDAR driver with real-time support"; // 模块文档字符串
    
    // 绑定RealtimeLidarClient类
    py::class_<rs_realtime::RealtimeLidarClient>(m, "Client")
        .def(py::init<>())
        .def("open",
             &rs_realtime::RealtimeLidarClient::open,
             py::arg("lidar_ip"),
             py::arg("save_path") = "")
        .def("get",
             &rs_realtime::RealtimeLidarClient::get_numpy,
             py::arg("return_intensity") = false,
             "Get point cloud data as numpy array with shape (N, 3) or (N, 4) containing [x, y, z] or [x, y, z, intensity]")
        .def("set_calib", &rs_realtime::RealtimeLidarClient::set_calib,
             "Set calibration parameters R (3x3) and t (3x1)")
        .def("stop", &rs_realtime::RealtimeLidarClient::stop,
             "Stop the LiDAR client");
    
    // 新增：PCAP逐帧读取器
    py::class_<PcapReader>(m, "PcapReader")
        .def(py::init<>())
        .def("open", &PcapReader::open, py::arg("pcap_path"), py::arg("save"))
        .def("set_calib", &PcapReader::set_calib, py::arg("R"), py::arg("t"))
        .def("set_ranges", &PcapReader::set_ranges, py::arg("ranges"))
        .def("get",
             &PcapReader::get,
             py::arg("return_intensity") = false,
             "Get one frame as numpy array with shape (N, 3) or (N, 4)")
        .def("stop", &PcapReader::stop);
}
