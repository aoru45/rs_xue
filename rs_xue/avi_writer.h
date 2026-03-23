#pragma once
#include "rs_driver/common/error_code.hpp"
#include "rs_driver/driver/driver_param.hpp"
#include <vector>
#include <string>
#include <memory>
#include <avcpp/av.h>
#include <avcpp/formatcontext.h>
#include <avcpp/frame.h>
#include <avcpp/codec.h>
#include <avcpp/codeccontext.h>
#include <avcpp/packet.h>
#include <avcpp/stream.h>
#include <string>
#include <atomic>
#include <mutex>
#include <rs_driver/msg/point_cloud_msg.hpp>

using namespace robosense::lidar;

typedef PointXYZIT PointT;
typedef PointCloudT<PointT> PointCloudMsg;

class AviWriter {
public:
    // 使用文件名与 fps 构造；width/height 将在第一次写入时从 PointCloudMsg 中获取
    AviWriter(const std::string& filename, int fps = 10);
    ~AviWriter();
    
    void write(std::shared_ptr<PointCloudMsg> frame);
    int getFrameCount() const { return frame_count_; }

private:
    std::string filename_;
    int width_{0};      // 点云宽（每段宽）
    int height_{0};     // 点云高（每段高）
    int fps_{10};

    std::unique_ptr<av::FormatContext> format_context_;
    std::unique_ptr<av::VideoEncoderContext> encoder_context_;
    av::Stream video_stream_;
    bool initialized_{false};
    int frame_count_{0};
    av::PixelFormat pixel_format_;
    std::mutex write_mutex_;
    
    void checkStaticLibraryEncoders();
    bool initialize(int width, int height);
    static std::pair<int, int> closest_factor_pair(int n);
    
    /**
     * @brief 从点云消息创建视频帧
     * @param frame 点云消息（包含 width/height/points）
     * @return avcpp视频帧
     */
    av::VideoFrame createVideoFrame(std::shared_ptr<PointCloudMsg> frame);
    
    /**
     * @brief 清理资源
     */
    void cleanup();
};
