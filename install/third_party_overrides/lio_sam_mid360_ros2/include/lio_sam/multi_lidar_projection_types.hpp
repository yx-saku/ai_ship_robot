#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>

namespace lio_sam
{

using CustomMsg = livox_ros_driver2::msg::CustomMsg;

struct RigidTransform
{
    bool identity{true};
    float r00{1.0F};
    float r01{0.0F};
    float r02{0.0F};
    float r10{0.0F};
    float r11{1.0F};
    float r12{0.0F};
    float r20{0.0F};
    float r21{0.0F};
    float r22{1.0F};
    float tx{0.0F};
    float ty{0.0F};
    float tz{0.0F};
};

struct InputLidarSpec
{
    std::string topic;
    int ring_offset{0};
};

struct MatchedLidarScan
{
    InputLidarSpec spec;
    CustomMsg::ConstSharedPtr cloud;
    double stamp_delta_sec{0.0};
    bool is_reference{false};
};

struct MatchedLidarScanGroup
{
    std_msgs::msg::Header header;
    rclcpp::Time reference_stamp;
    rclcpp::Time start_stamp;
    std::vector<MatchedLidarScan> scans;
    std::size_t point_count{0U};
};

struct MultiLidarQueueStats
{
    std::size_t queued_clouds{0U};
    std::size_t reference_queue_size{0U};
    unsigned long enqueued{0UL};
    unsigned long processed{0UL};
    unsigned long dropped{0UL};
    unsigned long matched{0UL};
    unsigned long missing{0UL};
};

}  // namespace lio_sam
