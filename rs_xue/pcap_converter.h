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
#include <array>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <limits>
#include <rs_driver/api/lidar_driver.hpp>
#include "avi_driver.h"
#include "avi_writer.h"

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

class PcapReader {
public:
    PcapReader();
    ~PcapReader();

    bool open(const std::string& pcap_path, std::string save_path="");
    bool init_writer(const std::string& avi_path);
    bool start();
    void stop();
    void set_calib(const py::array_t<float>& R, const py::array_t<float>& t);
    void set_ranges(const py::array_t<float>& ranges);

    pybind11::object get_point_xyz();
    pybind11::object get_point_xyzi();

private:
    std::unique_ptr<robosense::lidar::LidarDriver<PointCloudMsg>> driver_;
    std::unique_ptr<AviDriver<PointCloudMsg>> avi_driver_;
    std::unique_ptr<AviWriter> avi_writer_;
    robosense::lidar::RSDriverParam param_;

    // 内部队列（不使用全局队列，避免互相干扰）
    SyncQueue<std::shared_ptr<PointCloudMsg>> free_queue_;
    SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_queue_;

    // 处理线程
    std::thread processing_thread_;
    std::atomic<bool> should_stop_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> initialized_{false};

    // 最新一帧数据缓冲
    std::mutex data_mutex_;
    std::condition_variable data_cv_;
    bool has_new_{false};
    std::vector<float> latest_buf_; // 连续存放 [x,y,z,...]
    uint32_t latest_seq_{0};
    double latest_first_ts_{0.0};

    // 标定与范围
    std::array<float, 9> calib_R_ {1.f,0.f,0.f, 0.f,1.f,0.f, 0.f,0.f,1.f};
    std::array<float, 3> calib_t_ {0.f,0.f,0.f};
    bool has_calib_{false};

    std::array<float, 6> ranges_ { -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                                   -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                                   -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity() };
    bool has_ranges_{false};

    // 回调注册
    std::shared_ptr<PointCloudMsg> onGetPointCloud();
    void onReturnPointCloud(std::shared_ptr<PointCloudMsg> msg);
    void onException(const robosense::lidar::Error& code);

    // 后台处理
    void processingLoop();

    // 清理队列
    void cleanupQueues();
};

#endif // PCAP_CONVERTER_H