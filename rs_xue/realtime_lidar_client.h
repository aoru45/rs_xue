#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <queue>

// 添加pybind11头文件
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

// RoboSense SDK includes
#include <rs_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

using namespace robosense::lidar;
typedef PointXYZIT PointT;
typedef PointCloudT<PointT> PointCloudMsg;

namespace rs_realtime {

/**
 * @brief 点云数据结构，用于Python接口
 */
struct PointCloudData {
    std::vector<float> x;           // X坐标数组
    std::vector<float> y;           // Y坐标数组  
    std::vector<float> z;           // Z坐标数组
    std::vector<float> intensity;   // 强度数组
    std::vector<double> timestamp;  // 时间戳数组
    uint32_t frame_id;              // 帧ID
    size_t point_count;             // 点数量
    
    PointCloudData() : frame_id(0), point_count(0) {}
    
    void clear() {
        x.clear();
        y.clear();
        z.clear();
        intensity.clear();
        timestamp.clear();
        frame_id = 0;
        point_count = 0;
    }
};

/**
 * @brief RoboSense实时LiDAR客户端类
 * 
 * 这个类封装了RoboSense驱动的复杂性，提供类似Ouster Python SDK的简单接口。
 * 主要功能：
 * - 连接到RoboSense LiDAR传感器
 * - 实时获取点云数据
 * - 提供简单的get()方法获取最新一帧点云
 */
class RealtimeLidarClient {
public:
    /**
     * @brief 构造函数
     */
    RealtimeLidarClient();
    
    /**
     * @brief 析构函数
     */
    ~RealtimeLidarClient();
    
    /**
     * @brief 简化的初始化接口 - 只需要IP地址
     * 
     * @param lidar_ip 传感器的IP地址
     * @return true 初始化成功，false 初始化失败
     */
    bool initialize(const std::string& lidar_ip);
    
    /**
     * @brief 完整的初始化接口
     * 
     * @param lidar_ip 传感器的IP地址 
     * @param msop_port MSOP数据端口，默认6699
     * @param difop_port DIFOP数据端口，默认7788
     * @param lidar_type LiDAR类型，默认RSEM4
     * @param host_ip 主机IP地址，用于接收数据，默认"0.0.0.0"表示任意地址
     * @return true 初始化成功，false 初始化失败
     */
    bool initialize(const std::string& lidar_ip,
                   uint16_t msop_port,
                   uint16_t difop_port,
                   LidarType lidar_type,
                   const std::string& host_ip);
    
    /**
     * @brief 获取最新一帧点云数据
     */
    bool get(PointCloudData& point_cloud);
    
    /**
     * @brief 获取点云数据作为NumPy数组
     * @return pybind11::object NumPy数组或None
     */
    pybind11::object get_numpy();
    
    /**
     * @brief 获取点云数据并转换为适合Python的格式
     * 
     * @param data_ptr 输出参数，指向数据缓冲区的指针
     * @param point_count 输出参数，点的数量
     * @param has_nan 输出参数，是否检测到NaN值
     * @return true 成功获取数据，false 失败
     */
    bool get_numpy_data(float** data_ptr, size_t& point_count, bool& has_nan);
    
 
    
    /**
     * @brief 检查客户端是否已连接并正常工作
     * 
     * @return true 客户端正常工作，false 客户端未连接或出错
     */
    bool is_connected() const;
    
    /**
     * @brief 启动数据采集
     * 
     * @return true 启动成功，false 启动失败
     */
    bool start();
    
    /**
     * @brief 停止数据采集并断开连接
     */
    void stop();
    
    /**
     * @brief 强制停止LiDAR数据获取（用于异常退出情况）
     */
    void force_stop();
    
    /**
     * @brief 获取最后一次错误信息
     * 
     * @return 错误信息字符串
     */
    std::string get_last_error() const;

private:
    std::unique_ptr<LidarDriver<PointCloudMsg>> driver_;       // RoboSense驱动
    RSDriverParam param_;                                      // 驱动参数
    
    // 队列管理
    SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue_;    // 空闲点云队列
    SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue_; // 填充点云队列
    
    // 新增：后台处理线程和数据缓冲
    std::thread processing_thread_;                            // 后台处理线程
    std::atomic<bool> should_stop_processing_;                 // 处理线程停止标志
    
    // 最新点云数据存储（替换队列）
    PointCloudData latest_cloud_data_;                         // 最新的点云数据
    std::mutex cloud_data_mutex_;                              // 保护点云数据的互斥锁
    std::condition_variable cloud_data_cv_;                    // 条件变量，用于通知新数据到达
    bool has_new_data_;                                        // 标记是否有新数据
    
    // 状态管理
    std::atomic<bool> initialized_;                            // 初始化状态
    std::atomic<bool> running_;                                // 运行状态
    std::atomic<bool> connected_;                              // 连接状态
    
    // 错误处理
    mutable std::mutex error_mutex_;                           // 错误信息互斥锁
    std::string last_error_;                                   // 最后一次错误信息
    
    // 回调函数
    std::shared_ptr<PointCloudMsg> getPointCloudCallback();
    void returnPointCloudCallback(std::shared_ptr<PointCloudMsg> msg);
    void exceptionCallback(const Error& code);
    
    // 新增：后台处理线程函数
    void processCloudThread();
    
    // 数据转换函数
    void convertPointCloudMsg(const std::shared_ptr<PointCloudMsg>& msg, 
                             PointCloudData& point_cloud);
    
 
    
    // 工具函数
    void set_error(const std::string& error);
    void cleanup();
};

} // namespace rs_realtime