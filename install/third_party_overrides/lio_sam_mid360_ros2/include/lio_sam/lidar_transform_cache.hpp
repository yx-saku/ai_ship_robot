#pragma once

#include "multi_lidar_projection_types.hpp"

#include <optional>
#include <string>
#include <unordered_map>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>

namespace lio_sam
{

class LidarTransformCache
{
public:
    std::optional<RigidTransform> lookup(
        const std::string& target_frame,
        const std::string& source_frame,
        const builtin_interfaces::msg::Time& stamp,
        const double timeout_sec,
        tf2_ros::Buffer& tf_buffer,
        const rclcpp::Logger& logger,
        rclcpp::Clock& clock)
    {
        if (target_frame == source_frame)
        {
            RigidTransform transform;
            transform.identity = true;
            return transform;
        }

        const auto cache_key = target_frame + "<-" + source_frame;
        const auto cached = transform_cache_.find(cache_key);
        if (cached != transform_cache_.end())
            return cached->second;

        try
        {
            // LiDAR間の静的TFは初回だけ取得し、点ごとの処理ではfloat行列だけを使う。
            const auto stamped_transform = tf_buffer.lookupTransform(
                target_frame, source_frame, stamp, tf2::durationFromSec(timeout_sec));
            const auto& translation = stamped_transform.transform.translation;
            const auto& rotation = stamped_transform.transform.rotation;
            tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
            quaternion.normalize();
            tf2::Matrix3x3 matrix(quaternion);

            RigidTransform transform;
            transform.identity = false;
            transform.r00 = static_cast<float>(matrix[0][0]);
            transform.r01 = static_cast<float>(matrix[0][1]);
            transform.r02 = static_cast<float>(matrix[0][2]);
            transform.r10 = static_cast<float>(matrix[1][0]);
            transform.r11 = static_cast<float>(matrix[1][1]);
            transform.r12 = static_cast<float>(matrix[1][2]);
            transform.r20 = static_cast<float>(matrix[2][0]);
            transform.r21 = static_cast<float>(matrix[2][1]);
            transform.r22 = static_cast<float>(matrix[2][2]);
            transform.tx = static_cast<float>(translation.x);
            transform.ty = static_cast<float>(translation.y);
            transform.tz = static_cast<float>(translation.z);
            transform_cache_[cache_key] = transform;
            return transform;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                logger, clock, 2000,
                "Failed to transform CustomMsg from %s to %s: %s",
                source_frame.c_str(), target_frame.c_str(), ex.what());
            return std::nullopt;
        }
    }

private:
    std::unordered_map<std::string, RigidTransform> transform_cache_;
};

}  // namespace lio_sam
