#include "pcap_converter.h"

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
  RS_WARNING << code.toString() << RS_REND;
  exit(1);
}

void saveNpy(const std::string& path,
             const float* data,
             const std::vector<size_t>& shape)
{
    cnpy::npy_save(path, data, shape, "w");   // "w" = 覆盖写
}

void processCloud(const std::string& output_dir, int num_frames)
{
    while (true) {
        std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
        if (!msg) continue;
        const size_t N = msg->points.size();
        RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND; 
        std::vector<float> buf(N * 3);
        for (size_t i = 0; i < N; ++i) {
            buf[i*3+0] = msg->points[i].x;
            buf[i*3+1] = msg->points[i].y;
            buf[i*3+2] = msg->points[i].z;
        }
        std::ostringstream oss;
        oss << output_dir << "/cloud_"
            << std::setw(6) << std::setfill('0') << msg->seq << "_"
            << std::fixed << std::setprecision(6) << msg->points.front().timestamp
            << ".npy";
        saveNpy(oss.str(), buf.data(), {N, 3});
        free_cloud_queue.push(msg);
        if(msg->seq > num_frames) break;
    }
}

void processCloudWithCalib(const std::string& output_dir,
                           const float* R,
                           const float* t,
                           const float* ranges,
                           int num_frames)
{
    float x_min = ranges[0];
    float x_max = ranges[1];
    float y_min = ranges[2];
    float y_max = ranges[3];
    float z_min = ranges[4];
    float z_max = ranges[5];

    while (true)
    {
        std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();

        const size_t N = msg->points.size();
        RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;

        std::vector<float> buf;
        buf.reserve(N * 3);

        for (size_t i = 0; i < N; ++i)
        {
            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;

            float x_new = R[0] * x + R[1] * y + R[2] * z + t[0];
            float y_new = R[3] * x + R[4] * y + R[5] * z + t[1];
            float z_new = R[6] * x + R[7] * y + R[8] * z + t[2];

            if (x_new >= x_min && x_new <= x_max &&
                y_new >= y_min && y_new <= y_max &&
                z_new >= z_min && z_new <= z_max)
            {
                buf.push_back(x_new);
                buf.push_back(y_new);
                buf.push_back(z_new);
            }
        }

        if (!buf.empty())
        {
            std::ostringstream oss;
            oss << output_dir << "/cloud_"
                << std::setw(6) << std::setfill('0') << msg->seq << "_"
                << std::fixed << std::setprecision(6) << msg->points.front().timestamp
                << ".npy";
            saveNpy(oss.str(), buf.data(), {buf.size() / 3, 3});
        }else{
            RS_MSG << "msg: empty buffer" << RS_REND;
        }

        free_cloud_queue.push(msg);
        if (msg->seq > num_frames)
            break;
    }
}

int convert_pcap(const std::string& from_name, const std::string& to_name, int num_frames) {
  RS_TITLE << "------------------------------------------------------" << RS_REND;
  RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
  RS_TITLE << "------------------------------------------------------" << RS_REND;

  RSDriverParam param;  ///< Create a parameter object
  param.input_type = InputType::PCAP_FILE;
  param.input_param.pcap_path = from_name.c_str();  ///< Set the pcap file directory
  param.input_param.msop_port = 6699;                          ///< Set the lidar msop port number, the default is 6699
  param.input_param.pcap_repeat = false;
  param.input_param.difop_port = 7788;                         ///< Set the lidar difop port number, the default is 7788
  param.lidar_type = LidarType::RSEM4;                         ///< Set the lidar type. Make sure this type is correct
  param.print();
  LidarDriver<PointCloudMsg> driver;  ///< Declare the driver object
  driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback,
                               driverReturnPointCloudToCallerCallback);  ///< Register the point cloud callback
                                                                         ///< functions
  driver.regExceptionCallback(exceptionCallback);                        ///< Register the exception callback function
  if (!driver.init(param))                                               ///< Call the init function
  {
    RS_ERROR << "Driver Initialize Error..." << RS_REND;
    return -1;
  }
  std::thread cloud_handle_thread = std::thread(processCloud, to_name, num_frames);

  driver.start();  ///< The driver thread will start

  RS_DEBUG << "RoboSense Lidar-Driver Linux pcap demo start......" << RS_REND;
  cloud_handle_thread.join();
  driver.stop();
  return 0;
}

int convert_pcap_with_calib(const std::string& from_name,
                            const std::string& to_name,
                            const py::array_t<float>& R,
                            const py::array_t<float>& t,
                            const py::array_t<float>& ranges,
                            int num_frames)
{
    const float* R_data = static_cast<const float*>(R.request().ptr);
    const float* t_data = static_cast<const float*>(t.request().ptr);
    const float* ranges_data = static_cast<const float*>(ranges.request().ptr);

    RS_TITLE << "------------------------------------------------------" << RS_REND;
    RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
    RS_TITLE << "------------------------------------------------------" << RS_REND;

    RSDriverParam param;
    param.input_type = InputType::PCAP_FILE;
    param.input_param.pcap_path = from_name.c_str();
    param.input_param.msop_port = 6699;
    param.input_param.pcap_repeat = false;
    param.input_param.difop_port = 7788;
    param.lidar_type = LidarType::RSEM4;
    param.print();
    LidarDriver<PointCloudMsg> driver;
    driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback);
    driver.regExceptionCallback(exceptionCallback);
    if (!driver.init(param))
    {
        RS_ERROR << "Driver Initialize Error..." << RS_REND;
        return -1;
    }
    std::thread cloud_handle_thread = std::thread(processCloudWithCalib, to_name, R_data, t_data, ranges_data, num_frames);
    driver.start();
    RS_DEBUG << "RoboSense Lidar-Driver Linux pcap demo start......" << RS_REND;
    cloud_handle_thread.join();
    driver.stop();
    return 0;
}