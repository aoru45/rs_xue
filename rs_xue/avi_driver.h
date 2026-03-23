#pragma once

#include "rs_driver/common/error_code.hpp"
#include "rs_driver/driver/driver_param.hpp"
#include <avcpp/av.h>
#include <avcpp/formatcontext.h>
#include <avcpp/frame.h>
#include <avcpp/codec.h>
#include <avcpp/codeccontext.h>
#include <avcpp/packet.h>
#include <avcpp/stream.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <functional>
#include <memory>
#include <vector>
#include <cstring>
#include <cstdint>
#include "precise_sleep.h"

using namespace robosense::lidar;

/**
 * @brief AVI视频驱动，提供与LiDAR驱动相似的接口
 */
template <typename T_PointCloud>
class AviDriver
{
private:
    av::FormatContext format_context_;
    av::VideoDecoderContext decoder_context_;
    av::Stream video_stream_;
    int start_frame_index_{0};
    int current_frame_index_{0};
    std::atomic<bool> running_{false};
    std::thread worker_thread_;
    
    std::function<std::shared_ptr<T_PointCloud>(void)> cb_get_cloud_;
    std::function<void(std::shared_ptr<T_PointCloud>)> cb_put_cloud_;
    std::function<void(const Error&)> cb_exception_;
    
    std::chrono::milliseconds frame_interval_{100}; // 10 FPS = 100ms interval
    
    std::string avi_file_path_;

    std::pair<int, int> closest_factor_pair(int n) 
    {
        if (n < 1) return {1, 1};
        int root = static_cast<int>(std::sqrt(static_cast<double>(n)));
        while (root > 0 && n % root != 0) --root;
        if (root == 0) return {1, n}; 
        return {root, n / root};
    }

    /**
     * @brief AVI数据工作线程
     */
    void aviWorker()
    {
        av::VideoFrame frame;
        av::Packet packet;

        current_frame_index_ = start_frame_index_;
        
        while (running_) {
            auto current_time = std::chrono::steady_clock::now();
            if (1) {
                if (start_frame_index_ > 0) {
                    format_context_.seek(av::Timestamp(start_frame_index_, video_stream_.timeBase()));
                    start_frame_index_ = 0; // 只在第一次寻找
                }
                
                // 读取数据包
                packet = format_context_.readPacket();
                if (!packet) {
                    // 重新开始
                    format_context_.seek(av::Timestamp(0, video_stream_.timeBase()));
                    packet = format_context_.readPacket();
                    if (!packet) {
                        if (cb_exception_) {
                            Error error;
                            error.error_code_type = ErrCodeType::INFO_CODE;
                            cb_exception_(error);
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        continue;
                    }
                }
                
                // 使用decode方法解码数据包
                frame = decoder_context_.decode(packet);
                if (frame) {
                    // 将帧转换为点云
                    if (cb_get_cloud_ && cb_put_cloud_) {
                        auto cloud = cb_get_cloud_();
                        if (cloud && loadPointCloudFromFrame(frame, cloud)) {
                            cb_put_cloud_(cloud);
                            current_frame_index_++;
                        }
                    }
                }                
            }
            
            
            auto end_time = current_time + frame_interval_;
            if(end_time <= std::chrono::steady_clock::now()) {
                continue; // 已经过了该帧时间，直接处理下一帧
            }
            double duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - std::chrono::steady_clock::now()).count(); 
            precise_sleep(duration_us / 1e6);
            
        }
    }

    /**
     * @brief 从视频帧加载点云数据
     * @param frame avcpp视频帧
     * @param cloud 点云对象
     * @return 是否成功加载
     */
    bool loadPointCloudFromFrame(const av::VideoFrame& frame, std::shared_ptr<T_PointCloud>& cloud)
    {
        if (!frame.isValid()) {
            return false;
        }
        
        try {
            // 清空现有点
            cloud->points.clear();
            cloud->height = 1;
            cloud->is_dense = false;
            cloud->seq = current_frame_index_;
            
            //  (3*h, w, 3)
            // bgra ..
            
            const float scale = 1000.0f;
            const float center = 256.0f;
            
            // 获取帧数据和格式信息
            av::VideoFrame frame_bgr = frame;
            const uint8_t* frame_data = frame_bgr.data(0);
            int line_size = frame_bgr.raw()->linesize[0];
            int width = frame.width();
            int height = frame.height();
            auto pixel_format = frame.pixelFormat();
            // LOG_F(DEBUG, "Frame info: %dx%d, linesize=%d", width, height, line_size);
            //           << ", format=" << pixel_format.name() << std::endl;
            
            int h = height / 3;  // 高度被分成3部分
            int w = width;
            
            cloud->points.reserve(h * w);
            for (int i = 0; i < h; ++i) {
                for (int j = 0; j < w; ++j) {
                    typename T_PointCloud::PointT point;
                    std::vector<float> _v3(3, 0.0f);
                    for (int dim = 0; dim < 3; ++dim) {
                        int row_base = dim * h + i;
                        int pixel_offset = row_base * line_size + j * 4;
                        uint32_t high = frame_data[pixel_offset];     // B
                        uint32_t mid  = frame_data[pixel_offset + 1]; // G
                        uint32_t low  = frame_data[pixel_offset + 2]; // R
                        uint32_t val_mm = (high << 16) | (mid << 8) | low;
                        _v3[dim] = float(val_mm)/scale - center;
                    }
                    point.x = _v3[0];
                    point.y = _v3[1];
                    point.z = _v3[2];
                    // 从Z通道所在的段读取alpha作为强度（BGRA格式）
                    // 若alpha为0则强度设为0，否则设为alpha值
                    {
                        int row_base_alpha = 2 * h + i; // 第3段(Z)的当前行
                        int pixel_offset_alpha = row_base_alpha * line_size + j * 4;
                        uint8_t a = frame_data[pixel_offset_alpha + 3];
                        point.intensity = a;
                    }
                    
                    point.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    if(point.intensity > 0)
                        cloud->points.push_back(point);
                }
            }
            
            auto p = closest_factor_pair(cloud->points.size());
            cloud->width = static_cast<uint32_t>(p.first);
            cloud->height = static_cast<uint32_t>(p.second);
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Error loading point cloud from frame: " << e.what() << std::endl;
            return false;
        }
    }

public:
    AviDriver() = default;
    
    ~AviDriver()
    {
        stop();
    }

    /**
     * @brief 注册点云回调函数
     * @param cb_get_cloud 获取空闲点云的回调函数
     * @param cb_put_cloud 返回填充点云的回调函数
     */
    void regPointCloudCallback(
        const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_cloud,
        const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_cloud)
    {
        cb_get_cloud_ = cb_get_cloud;
        cb_put_cloud_ = cb_put_cloud;
    }

    /**
     * @brief 注册异常回调函数
     * @param cb_excep 异常处理回调函数
     */
    void regExceptionCallback(const std::function<void(const Error&)>& cb_excep)
    {
        cb_exception_ = cb_excep;
    }

    /**
     * @brief 初始化驱动
     * @param param 驱动参数
     * @return 初始化是否成功
     */
    bool init(const RSDriverParam& param)
    {
        // 从参数中获取AVI文件路径
        avi_file_path_ = param.input_param.pcap_path; // 复用pcap_path字段
        
        try {
            // 打开格式上下文
            format_context_.openInput(avi_file_path_);
            format_context_.findStreamInfo();
            
            // 查找视频流
            for (size_t i = 0; i < format_context_.streamsCount(); ++i) {
                auto stream = format_context_.stream(i);
                if (stream.mediaType() == AVMEDIA_TYPE_VIDEO) {
                    video_stream_ = stream;
                    break;
                }
            }
            
            if (!video_stream_.isValid()) {
                std::cerr << "No video stream found in file: " << avi_file_path_ << std::endl;
                return false;
            }
            
            // 初始化解码器
            auto codec = av::findDecodingCodec(video_stream_.codecParameters().codecId());
            decoder_context_ = av::VideoDecoderContext(video_stream_, codec);
            decoder_context_.open(codec);
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize AVI driver: " << e.what() << std::endl;
            return false;
        }
    }

    /**
     * @brief 启动驱动
     * @return 启动是否成功
     */
    bool start()
    {
        if (running_) {
            return true;
        }
        
        running_ = true;
        worker_thread_ = std::thread(&AviDriver::aviWorker, this);
        
        return true;
    }

    /**
     * @brief 停止驱动
     */
    void stop()
    {
        running_ = false;
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        // avcpp对象会自动清理，不需要手动释放
    }
};
