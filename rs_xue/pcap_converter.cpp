#include "pcap_converter.h"
#include <cstring>

// 全局队列定义
SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver,
  //       so please DO NOT do time-consuming task here.
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver,
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud()
  //       below)
  stuffed_cloud_queue.push(msg);
}

void exceptionCallback(const Error& code)
{
  // Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the
  // driver,
  //       so please DO NOT do time-consuming task here.
  RS_WARNING << code.toString() + " in pcap_converter" << RS_REND;
//   exit(1);
}



// =============== PcapReader 实现 ===============
PcapReader::PcapReader()
    : driver_(std::make_unique<robosense::lidar::LidarDriver<PointCloudMsg>>()),
      avi_driver_(std::make_unique<AviDriver<PointCloudMsg>>()),
      initialized_(false),
      running_(false),
      should_stop_(false),
      has_calib_(false),
      has_ranges_(false) {}


PcapReader::~PcapReader() { stop(); }

bool PcapReader::open(const std::string& pcap_path, std::string save_path) {
    try {
        if(save_path != ""){
            init_writer(save_path);
        }
        param_.input_param.pcap_path = pcap_path.c_str();
        if(pcap_path.substr(pcap_path.size() - 4) == ".avi"){
            avi_driver_ = std::make_unique<AviDriver<PointCloudMsg>>();
            param_.input_param.pcap_path = pcap_path; // AVI驱动复用pcap_path字段
            RS_MSG << "Using AVI driver for file: " << pcap_path << RS_REND;
            avi_driver_->regPointCloudCallback(
                [this]() { return this->onGetPointCloud(); },
                [this](std::shared_ptr<PointCloudMsg> msg) { this->onReturnPointCloud(msg); }
            );

            avi_driver_->regExceptionCallback(
                [this](const Error& code) { this->onException(code); }
            );

            // 初始化AVI驱动
            if (!avi_driver_->init(param_)) {
                return false;
            }
            initialized_ = true;
            return start();
        }else{
            param_.input_type = InputType::PCAP_FILE;
            param_.input_param.msop_port = 6699;
            param_.input_param.difop_port = 7788;
            param_.input_param.pcap_repeat = false;
            param_.decoder_param.wait_for_difop = false;
            param_.lidar_type = LidarType::RSEM4;

            driver_->regPointCloudCallback(
                [this]() { return this->onGetPointCloud(); },
                [this](std::shared_ptr<PointCloudMsg> msg) { this->onReturnPointCloud(msg); }
            );
            driver_->regExceptionCallback([this](const robosense::lidar::Error& code) { this->onException(code); });

            if (!driver_->init(param_)) {
                RS_ERROR << "PcapReader: Driver init failed" << RS_REND;
                return false;
            }
            initialized_ = true;
            return start();

        }
        
    } catch (...) {
        return false;
    }
}

bool PcapReader::start() {
    if (!initialized_) return false;
    if (running_) return true;

    should_stop_ = false;
    driver_->start();
    running_ = true;
    return true;
}

void PcapReader::stop() {
    if (!running_) return;

    should_stop_ = true;
    // 确保任何 popWait 都能被唤醒
    stuffed_queue_.push(std::shared_ptr<PointCloudMsg>());
    try { driver_->stop(); } catch (...) {}
    try { avi_driver_->stop(); } catch (...) {}
    try { avi_writer_.reset(); } catch (...) {}
    cleanupQueues();
    running_ = false;
    initialized_ = false;
}

void PcapReader::set_calib(const py::array_t<float>& R, const py::array_t<float>& t) {
    const float* R_data = static_cast<const float*>(R.request().ptr);
    const float* t_data = static_cast<const float*>(t.request().ptr);
    calib_R_ = { R_data[0], R_data[1], R_data[2],
                 R_data[3], R_data[4], R_data[5],
                 R_data[6], R_data[7], R_data[8] };
    calib_t_ = { t_data[0], t_data[1], t_data[2] };
    has_calib_ = true;
}

void PcapReader::set_ranges(const py::array_t<float>& ranges) {
    const float* r = static_cast<const float*>(ranges.request().ptr);
    ranges_ = { r[0], r[1], r[2], r[3], r[4], r[5] };
    has_ranges_ = true;
}

std::shared_ptr<PointCloudMsg> PcapReader::onGetPointCloud() {
    auto msg = free_queue_.pop();
    if (msg) return msg;
    return std::make_shared<PointCloudMsg>();
}

void PcapReader::onReturnPointCloud(std::shared_ptr<PointCloudMsg> msg) {
    while(!stuffed_queue_.empty()){
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    stuffed_queue_.push(msg);
}

void PcapReader::onException(const robosense::lidar::Error& code) {
    RS_WARNING << "PcapReader Exception: " << code.toString() << " nothing" << RS_REND;
    // 当PCAP读取完成（Info_PcapExit）或出现非信息级错误时，通知退出
    // const std::string s = code.toString();
    // if (s.find("Info_PcapExit") != std::string::npos) {
    //     should_stop_ = true;
    //     // 唤醒等待的 get()
    //     data_cv_.notify_all();
    //     // 推送一个空消息以唤醒 popWait 立即退出
    //     stuffed_queue_.push(std::shared_ptr<PointCloudMsg>());
    // }
}

// processingLoop 已移除：直接在 get() 中逐帧处理，避免覆盖/跳帧

pybind11::object PcapReader::get(bool return_intensity) {
    if (should_stop_) return pybind11::none();

    // 直接阻塞等待下一帧（不做后台聚合），保证一帧不丢
    auto msg = stuffed_queue_.popWait();
    if (!msg) {
        // 可能是退出信号
        return pybind11::none();
    }
    if(avi_writer_){
        avi_writer_->write(msg);
    }

    const size_t N = msg->points.size();
    const size_t width = return_intensity ? 4 : 3;
    std::vector<float> buf;
    buf.reserve(N * width);

    for (size_t i = 0; i < N; ++i) {
        const auto& p = msg->points[i];
        float x = p.x, y = p.y, z = p.z;
        float x_new = x, y_new = y, z_new = z;
        if (has_calib_) {
            x_new = calib_R_[0]*x + calib_R_[1]*y + calib_R_[2]*z + calib_t_[0];
            y_new = calib_R_[3]*x + calib_R_[4]*y + calib_R_[5]*z + calib_t_[1];
            z_new = calib_R_[6]*x + calib_R_[7]*y + calib_R_[8]*z + calib_t_[2];
        }
        if (has_ranges_) {
            if (x_new < ranges_[0] || x_new > ranges_[1] ||
                y_new < ranges_[2] || y_new > ranges_[3] ||
                z_new < ranges_[4] || z_new > ranges_[5]) {
                continue;
            }
        }
        buf.push_back(x_new);
        buf.push_back(y_new);
        buf.push_back(z_new);
        if (return_intensity) {
            buf.push_back(static_cast<float>(p.intensity));
        }
    }

    size_t count = buf.size() / width;
    auto arr = pybind11::array_t<float>({ static_cast<pybind11::ssize_t>(count), static_cast<pybind11::ssize_t>(width) });
    auto view = arr.request();
    float* ptr = static_cast<float*>(view.ptr);
    if (!buf.empty()) {
        std::memcpy(ptr, buf.data(), buf.size() * sizeof(float));
    }

    // 归还消息供驱动复用
    free_queue_.push(msg);

    return arr;
}

bool PcapReader::init_writer(const std::string& avi_path){
    avi_writer_ = std::make_unique<AviWriter>(avi_path, 10);
    return true;
}
void PcapReader::cleanupQueues() {
    while (auto m = free_queue_.pop()) {}
    while (auto m = stuffed_queue_.pop()) {}
}
