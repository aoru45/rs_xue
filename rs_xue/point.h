#pragma once

#include <rs_driver/msg/point_cloud_msg.hpp>



typedef PointXYZIT PointT;
typedef PointCloudT<PointT> PointCloudMsg;

struct TensorMsg{
    float* points; // (M, 4)
    uint32_t height = 0;    ///< Height of point cloud
    uint32_t width = 0;     ///< Width of point cloud
    bool is_dense = false;  ///< If is_dense is true, the point cloud does not contain NAN points,
    double timestamp = 0.0;
    uint32_t seq = 0;           ///< Sequence number of message
    std::string name = "";
};

