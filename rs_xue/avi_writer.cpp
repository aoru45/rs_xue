#include "avi_writer.h"
#include <string>
#include <algorithm>
#include <cmath>
#include <utility>
#include <cstring>


AviWriter::AviWriter(const std::string& filename, int fps)
    : filename_(filename), fps_(fps)
{
    pixel_format_ = av::PixelFormat("bgra");
    std::cout << "AviWriter constructed: file=" << filename_ << " fps=" << fps_ << " pixfmt=bgra" << std::endl;
}

AviWriter::~AviWriter()
{
    cleanup();
}

void AviWriter::write(std::shared_ptr<PointCloudMsg> frame)
{
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (!frame) {
        std::cout << "AviWriter::write called with null frame" << std::endl;
        return;
    }
    if (!initialized_) {
        std::pair<int, int> dims = AviWriter::closest_factor_pair(624000);
        int w = dims.first;
        int h = dims.second;
        std::cout << "AviWriter::write initializing with cloud dims: " << w << " x " << h << std::endl;


        if (w <= 0 || h <= 0) {
            std::cout << "AVI writer: invalid width/height after reshape: " << w << " x " << h << std::endl;
            return;
        }
        if (!initialize(w, h)) {
            std::cout << "AVI writer failed to initialize with cloud dims" << std::endl;
            return;
        }
    }
    
    try {
        av::VideoFrame video_frame = createVideoFrame(frame);
            if (!video_frame.isValid()) {
            std::cout << "Failed to create video frame from tensor" << std::endl;
            return;
        }
        
        av::Rational time_base(1, fps_);
        video_frame.setTimeBase(time_base);
        video_frame.setPts(av::Timestamp(frame_count_, time_base));
        
        {
            av::Packet packet = encoder_context_->encode(video_frame);
            if (packet) {
                format_context_->writePacket(packet);
                frame_count_++;
            }
        }
        
    } catch (const std::exception& e) {
        std::cout << "Error writing frame to AVI: " << e.what() << std::endl;
    }
}

void AviWriter::checkStaticLibraryEncoders() {
    std::cout << "=== Checking Static Library Encoders ===" << std::endl;
    
    const AVCodec* codec = nullptr;
    void* opaque = nullptr;
    
    int encoder_count = 0;
    while ((codec = av_codec_iterate(&opaque))) {
        if (av_codec_is_encoder(codec) && codec->type == AVMEDIA_TYPE_VIDEO) {
            encoder_count++;
            std::cout << "Encoder: " << codec->name << " (ID: " << codec->id << ")" << std::endl;
        }
    }
    
    const AVCodec* h264_encoders[] = {
        avcodec_find_encoder_by_name("libx264"),
        avcodec_find_encoder_by_name("h264"),
        avcodec_find_encoder(AV_CODEC_ID_H264)
    };
    
    for (int i = 0; i < 3; i++) {
        if (h264_encoders[i]) {
            std::cout << "H.264 encoder found: " << h264_encoders[i]->name << std::endl;
        }
    }

    std::cout << "========================================" << std::endl;
}


bool AviWriter::initialize(int width, int height)
{
    // checkStaticLibraryEncoders();
    try {
        format_context_ = std::make_unique<av::FormatContext>();
        format_context_->openOutput(filename_);
        auto codec = av::findEncodingCodec(AV_CODEC_ID_FFV1);
        if (!av_codec_is_encoder(codec.raw())) {
            std::cout << "Codec is not an encoder: " << codec.name() << std::endl;
            return false;
        }
        
        width_ = width;
        height_ = height;

        encoder_context_ = std::make_unique<av::VideoEncoderContext>();

        encoder_context_->setCodec(codec);
        encoder_context_->setWidth(width_);
        encoder_context_->setHeight(3 * height_);
        // encoder_context_->setHeight(height_);
        encoder_context_->setPixelFormat(pixel_format_);

        encoder_context_->setTimeBase(av::Rational(1, fps_));
        
        std::error_code ec;
        encoder_context_->open(codec, ec);
        if (ec) {
            std::cout << "Failed to open encoder: " << ec.message() << std::endl;
            return false;
        }
        video_stream_ = format_context_->addStream(*encoder_context_);
        if (!video_stream_.isValid()) {
            std::cout << "Failed to add stream" << std::endl;
            return false;
        }
        video_stream_.setTimeBase(av::Rational(1, fps_));


        format_context_->writeHeader();
        
        initialized_ = true;
        std::string pixel_format_name = pixel_format_.name();
        return true;
        
    } catch (const std::exception& e) {
        std::cout << "Failed to initialize AVI writer: " << e.what() << std::endl;
        return false;
    }
}

av::VideoFrame AviWriter::createVideoFrame(std::shared_ptr<PointCloudMsg> frame)
{
    try {
        av::VideoFrame video_frame(pixel_format_, width_, 3 * height_);
        // av::VideoFrame video_frame(pixel_format_, width_, height_);
        uint8_t* frame_data = video_frame.data(0);
        int line_size = video_frame.raw()->linesize[0];
        
        const float scale = 1000.0f;
        const float center = 256.0f;
        
        int h = height_; 
        int w = width_;   
        int total_height = 3 * h; 
        // int total_height = h;
        
        std::memset(frame_data, 0, line_size * total_height);

        for (int i = 0; i < h; ++i) {
            for (int j = 0; j < w; ++j) {
                int point_idx = i * w + j;
                if (point_idx >= static_cast<int>(frame->points.size())) {
                    continue;
                }

                float x = frame->points[point_idx].x;
                float y = frame->points[point_idx].y;
                float z = frame->points[point_idx].z;
                
                uint8_t intensity = static_cast<uint8_t>(frame->points[point_idx].intensity);
                
                uint32_t val_x = static_cast<uint32_t>((x + center) * scale);
                uint32_t val_y = static_cast<uint32_t>((y + center) * scale);
                uint32_t val_z = static_cast<uint32_t>((z + center) * scale);

                // uint8_t val_x = static_cast<uint8_t>((x + center) * scale);
                // uint8_t val_y = static_cast<uint8_t>((y + center) * scale);
                // uint8_t val_z = static_cast<uint8_t>((z + center) * scale);

                val_x = std::min(std::max(val_x, 0u), 0xFFFFFFu);
                val_y = std::min(std::max(val_y, 0u), 0xFFFFFFu);
                val_z = std::min(std::max(val_z, 0u), 0xFFFFFFu);

                for (int dim = 0; dim < 3; ++dim) {
                    int row_base = dim * h + i;
                    int pixel_offset = row_base * line_size + j * 4;
                    
                    uint32_t value = 0;
                    switch (dim) {
                        case 0: value = val_x; break; 
                        case 1: value = val_y; break; 
                        case 2: value = val_z; break; 
                    }
                    
                    uint8_t high = (value >> 16) & 0xFF;  
                    uint8_t mid  = (value >> 8) & 0xFF;   
                    uint8_t low  = value & 0xFF;          
                    
                    frame_data[pixel_offset] = high;      
                    frame_data[pixel_offset + 1] = mid;   
                    frame_data[pixel_offset + 2] = low;   
                    frame_data[pixel_offset + 3] = (dim == 2) ? intensity : 255; 
                }
                // int pixel_offset = i * line_size + j * 4;
                // frame_data[pixel_offset] = val_x;          
                // frame_data[pixel_offset + 1] = val_y;      
                // frame_data[pixel_offset + 2] = val_z;      
                // frame_data[pixel_offset + 3] = intensity;
            }
        }
        return video_frame;
        
    } catch (const std::exception& e) {
        std::cout << "Failed to create video frame from tensor: " << e.what() << std::endl;
        return av::VideoFrame();
    }
}

void AviWriter::cleanup()
{
    if (!initialized_) {
        return;
    }
    
    try {

        std::lock_guard<std::mutex> lock(write_mutex_);
        int flushed = 0;
        while (true) {
            av::Packet packet = encoder_context_->encode();
            if (!packet) break;
            format_context_->writePacket(packet);
            flushed++;
        }
        std::cout << "AviWriter::cleanup flushed " << flushed << " packets" << std::endl;
        
        format_context_->writeTrailer();
          std::cout << "AVI writer closed successfully. Wrote " << frame_count_ << " frames to " << filename_ << std::endl;
        initialized_ = false;
        
    } catch (const std::exception& e) {
        std::cout << "Error during AVI writer cleanup: " << e.what() << std::endl;
    }
}

std::pair<int, int> AviWriter::closest_factor_pair(int n) 
{
    if (n < 1) return {1, 1};
    int root = static_cast<int>(std::sqrt(static_cast<double>(n)));
    while (root > 0 && n % root != 0) --root;
    if (root == 0) return {1, n}; 
    return {root, n / root};
}
