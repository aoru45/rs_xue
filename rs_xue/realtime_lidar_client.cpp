#include "realtime_lidar_client.h"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <cstring>
#include <limits>
#include <algorithm>

namespace py = pybind11;
#include <iostream>
#include <chrono>
#include <iomanip>
#include <cmath>

namespace rs_realtime {

// 构造函数
RealtimeLidarClient::RealtimeLidarClient() 
    : driver_(std::make_unique<LidarDriver<PointCloudMsg>>()),
      initialized_(false),
      running_(false), 
      connected_(false),
      should_stop_processing_(false),
      has_new_data_(false) {
}

void RealtimeLidarClient::processCloudThread() {
    while (!should_stop_processing_) {
        std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue_.popWait();
        if (!msg) {
            continue;
        }
        
        // 添加与demo_online.cpp相同的调试打印，检查xyz三个坐标
        size_t N = msg->points.size();
       
        
        // 转换点云数据
        PointCloudData cloud_data;
        convertPointCloudMsg(msg, cloud_data);
        
        // 更新最新数据（加锁保护）
        {
            std::lock_guard<std::mutex> lock(cloud_data_mutex_);
            latest_cloud_data_ = std::move(cloud_data);
            has_new_data_ = true;
        }
        cloud_data_cv_.notify_one();  // 通知等待的get函数
        
        // 回收消息到空闲队列
        free_cloud_queue_.push(msg);
    }
}

bool RealtimeLidarClient::get(PointCloudData& point_cloud) {
    if (!running_) {
        set_error("Client is not running");
        return false; 
    }
    
    std::unique_lock<std::mutex> lock(cloud_data_mutex_);
    
    // 等待新数据到达
    cloud_data_cv_.wait(lock, [this] { return has_new_data_ || should_stop_processing_; });
    
    if (should_stop_processing_) {
        return false;
    }
    
    if (has_new_data_) {
        point_cloud = std::move(latest_cloud_data_);
        latest_cloud_data_.clear();  // 清空已移动的数据
        has_new_data_ = false;
        return true;
    }
    
    return false;
}

void RealtimeLidarClient::stop() {
    if (!running_) {
        return;
    }
    
    // 停止处理线程
    should_stop_processing_ = true;
    cloud_data_cv_.notify_all();  // 唤醒所有等待的线程
    
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    
    // 停止驱动
    if (driver_) {
        driver_->stop();
    }
    
    running_ = false;
    connected_ = false;
    
    RS_MSG << "RealtimeLidarClient stopped" << RS_REND;
}

// 析构函数
RealtimeLidarClient::~RealtimeLidarClient() {
    // 确保在析构时强制停止所有操作
    force_stop();
}


bool RealtimeLidarClient::initialize(const std::string& lidar_ip) {
    // 使用默认参数：端口6699/7788，LiDAR类型RS16，自动检测本机IP
    return initialize(lidar_ip, 6699, 7788, LidarType::RSEM4, "0.0.0.0");
}

bool RealtimeLidarClient::initialize(const std::string& lidar_ip,
                                   uint16_t msop_port,
                                   uint16_t difop_port,
                                   LidarType lidar_type,
                                   const std::string& host_ip) {
    try {
        // 配置驱动参数
        param_.input_type = InputType::ONLINE_LIDAR;           // 关键：设置为在线模式
        param_.input_param.host_address = host_ip;             // 主机IP地址
        param_.input_param.msop_port = msop_port;              // MSOP端口
        param_.input_param.difop_port = difop_port;            // DIFOP端口
        param_.lidar_type = lidar_type;                        // LiDAR类型

        param_.decoder_param.dense_points = true;
        
        // 如果提供了LiDAR IP，设置组播相关参数
        if (!lidar_ip.empty() && lidar_ip != "192.168.1.200") {
            param_.input_param.group_address = lidar_ip;       // 组播地址
        }
        
        // 打印配置信息
        RS_TITLE << "------------------------------------------------------" << RS_REND;
        RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
        RS_TITLE << "------------------------------------------------------" << RS_REND;
        param_.print();
        
        // 注册回调函数
        driver_->regPointCloudCallback(
            [this]() { return this->getPointCloudCallback(); },
            [this](std::shared_ptr<PointCloudMsg> msg) { this->returnPointCloudCallback(msg); }
        );
        
        driver_->regExceptionCallback(
            [this](const Error& code) { this->exceptionCallback(code); }
        );
        
        // 初始化驱动
        if (!driver_->init(param_)) {
            set_error("Driver initialization failed");
            return false;
        }
        
        initialized_ = true;
        connected_ = true;
        
        RS_MSG << "RealtimeLidarClient initialized successfully" << RS_REND;

        start();
        return true;
        
    } catch (const std::exception& e) {
        set_error(std::string("Initialization exception: ") + e.what());
        return false;
    }
}

bool RealtimeLidarClient::start() {
    if (!initialized_) {
        return false;
    }
    if (running_) {
        return true;  // 已经在运行，直接返回
    }

    
    // 启动LiDAR驱动
    driver_->start();  // 修复：使用 -> 而不是 .
    
    // 启动后台处理线程
    should_stop_processing_ = false;
    processing_thread_ = std::thread(&RealtimeLidarClient::processCloudThread, this);
    running_ = true;
    
    return true;
}

bool RealtimeLidarClient::is_connected() const {
    return connected_ && running_;
}

void RealtimeLidarClient::force_stop() {
    // 强制停止，用于异常退出情况
    try {
        // 立即设置所有状态为false
        running_ = false;
        connected_ = false;
        initialized_ = false;
        
        // 尝试停止驱动，但不抛出异常
        if (driver_) {
            try {
                driver_->stop();
            } catch (...) {
                // 忽略停止过程中的异常
            }
        }
        
        // 强制清理资源
        cleanup();
        
        RS_MSG << "RealtimeLidarClient force stopped" << RS_REND;
    } catch (...) {
        // 强制停止不应该抛出任何异常
    }
}

std::string RealtimeLidarClient::get_last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

// 私有方法实现

std::shared_ptr<PointCloudMsg> RealtimeLidarClient::getPointCloudCallback() {
    // 从空闲队列获取点云消息
    std::shared_ptr<PointCloudMsg> msg = free_cloud_queue_.pop();
    if (msg.get() != nullptr) {
        return msg;
    }
    
    // 如果没有空闲消息，创建新的
    return std::make_shared<PointCloudMsg>();
}

void RealtimeLidarClient::returnPointCloudCallback(std::shared_ptr<PointCloudMsg> msg) {
    // 清空队列中的所有旧数据
    // while (true) {
    //     auto old_msg = stuffed_cloud_queue_.pop(); // 非阻塞pop
    //     if (!old_msg) {
    //         break; // 队列已空
    //     }
    //     // 将旧消息放回空闲队列
    //     free_cloud_queue_.push(old_msg);
    // }
    
    // 将新的点云消息放入队列
    stuffed_cloud_queue_.push(msg);
}

void RealtimeLidarClient::exceptionCallback(const Error& code) {
    RS_WARNING << "LiDAR Exception: " << code.toString() << RS_REND;
    set_error("LiDAR Exception: " + code.toString());
    
    // 对于严重错误，断开连接
    if (code.error_code_type == ErrCodeType::INFO_CODE) {
        // 信息级别，不需要断开
    } else {
        connected_ = false;
    }
}

void RealtimeLidarClient::convertPointCloudMsg(const std::shared_ptr<PointCloudMsg>& msg, 
                                             PointCloudData& point_cloud) {
    point_cloud.clear();
    
    const size_t N = msg->points.size();
    if (N == 0) {
        return;
    }
    
    // 预分配内存
    point_cloud.x.reserve(N);
    point_cloud.y.reserve(N);
    point_cloud.z.reserve(N);
    point_cloud.intensity.reserve(N);
    point_cloud.timestamp.reserve(N);
    
    for (size_t i = 0; i < N; ++i) {
        const auto& point = msg->points[i];
        
        point_cloud.x.push_back(-point.y);
        point_cloud.y.push_back(point.x);
        point_cloud.z.push_back(point.z);
        point_cloud.intensity.push_back(point.intensity);
        point_cloud.timestamp.push_back(point.timestamp);
    }
    
    point_cloud.frame_id = msg->seq;
    point_cloud.point_count = N;
    
}


void RealtimeLidarClient::set_error(const std::string& error) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error;
    RS_ERROR << error << RS_REND;
}

void RealtimeLidarClient::cleanup() {
    // 清空队列 - 使用pop()直到返回nullptr
    while (true) {
        auto msg = free_cloud_queue_.pop();
        if (!msg) break;
    }
    while (true) {
        auto msg = stuffed_cloud_queue_.pop();
        if (!msg) break;
    }
    
    initialized_ = false;
    connected_ = false;
}

// 将get_numpy方法移到namespace内部
py::object RealtimeLidarClient::get_numpy() {
    PointCloudData cloud_data;
    if (!get(cloud_data)) {
        return py::none();
    }
    
    size_t point_count = cloud_data.point_count;
    if (point_count == 0) {
        return py::none();
    }
    
    // 直接创建NumPy数组，避免不必要的检查
    auto result = py::array_t<float>({
        static_cast<py::ssize_t>(point_count), 
        static_cast<py::ssize_t>(3)
    });
    
    auto buf = result.request();
    float* ptr = static_cast<float*>(buf.ptr);
    
    // 高效的内存拷贝，避免逐个元素赋值
    for (size_t i = 0; i < point_count; ++i) {
        ptr[i * 3 + 0] = cloud_data.x[i];
        ptr[i * 3 + 1] = cloud_data.y[i];
        ptr[i * 3 + 2] = cloud_data.z[i];
    }
    
    return result;
}

} // namespace rs_realtime