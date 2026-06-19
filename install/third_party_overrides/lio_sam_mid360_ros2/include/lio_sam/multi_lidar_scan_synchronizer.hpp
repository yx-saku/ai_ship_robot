#pragma once

#include "multi_lidar_projection_types.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/time.hpp>

namespace lio_sam
{

class MultiLidarScanSynchronizer
{
public:
    MultiLidarScanSynchronizer(
        std::vector<InputLidarSpec> specs,
        std::string reference_topic,
        const double max_stamp_delta_sec,
        const std::size_t max_queue_size)
        : specs_(std::move(specs)),
          reference_topic_(std::move(reference_topic)),
          max_stamp_delta_sec_(max_stamp_delta_sec),
          max_queue_size_(max_queue_size)
    {
        for (const auto& spec : specs_)
            queues_.try_emplace(spec.topic);
    }

    void enqueue(const std::string& topic, const CustomMsg::ConstSharedPtr& message)
    {
        auto iterator = queues_.find(topic);
        if (iterator == queues_.end() || message == nullptr)
            return;

        // callback側はshared_ptrを保存するだけにし、巨大なCustomMsgコピーを避ける。
        iterator->second.push_back(message);
        enforceQueueLimit(iterator->second);
        ++enqueued_count_;
    }

    bool buildNextGroup(MatchedLidarScanGroup* group)
    {
        if (group == nullptr)
            return false;

        const auto reference_iterator = queues_.find(reference_topic_);
        if (reference_iterator == queues_.end() || reference_iterator->second.empty())
            return false;
        if (!flush_oldest_reference_ && specs_.size() > 1U && reference_iterator->second.size() < 2U)
            return false;

        const auto& reference_cloud = reference_iterator->second.front();
        if (reference_cloud == nullptr)
            return false;

        // 基準scanのstampで各LiDARの最近傍scanを選び、欠落LiDARは今回だけ外す。
        group->header = reference_cloud->header;
        group->reference_stamp = rclcpp::Time(reference_cloud->header.stamp);
        group->scans.clear();
        group->point_count = 0U;
        group->scans.reserve(specs_.size());
        for (const auto& spec : specs_)
        {
            if (spec.topic == reference_topic_)
            {
                group->scans.push_back(MatchedLidarScan{spec, reference_cloud, 0.0, true});
                group->point_count += reference_cloud->points.size();
                continue;
            }

            const auto matched_cloud = findBestMatchingCloud(spec.topic, group->reference_stamp);
            if (!matched_cloud.has_value())
                continue;

            const auto stamp_delta_sec =
                (rclcpp::Time(matched_cloud.value()->header.stamp) - group->reference_stamp).seconds();
            group->scans.push_back(MatchedLidarScan{spec, matched_cloud.value(), stamp_delta_sec, false});
            group->point_count += matched_cloud.value()->points.size();
        }

        return !group->scans.empty();
    }

    void markProcessed(const MatchedLidarScanGroup& group)
    {
        ++processed_count_;
        recordMatchStats(group);
        pruneProcessedGroup(group.reference_stamp);
    }

    void markDropped(const MatchedLidarScanGroup& group)
    {
        ++dropped_count_;
        recordMatchStats(group);
        pruneProcessedGroup(group.reference_stamp);
    }

    void requestFlushOldestReference()
    {
        flush_oldest_reference_ = true;
        if (referenceQueueSize() == 0U)
            clearNonReferenceQueues();
    }

    MultiLidarQueueStats stats() const
    {
        MultiLidarQueueStats stats;
        for (const auto& [topic, queue] : queues_)
        {
            (void)topic;
            stats.queued_clouds += queue.size();
        }
        const auto reference_iterator = queues_.find(reference_topic_);
        if (reference_iterator != queues_.end())
            stats.reference_queue_size = reference_iterator->second.size();
        stats.enqueued = enqueued_count_;
        stats.processed = processed_count_;
        stats.dropped = dropped_count_;
        stats.matched = matched_count_;
        stats.missing = missing_count_;
        return stats;
    }

private:
    std::optional<CustomMsg::ConstSharedPtr> findBestMatchingCloud(
        const std::string& topic,
        const rclcpp::Time& reference_stamp) const
    {
        const auto iterator = queues_.find(topic);
        if (iterator == queues_.end() || iterator->second.empty())
            return std::nullopt;

        CustomMsg::ConstSharedPtr best_cloud;
        double best_delta_sec = std::numeric_limits<double>::max();
        for (const auto& candidate : iterator->second)
        {
            if (candidate == nullptr)
                continue;

            const auto delta_sec =
                std::abs((rclcpp::Time(candidate->header.stamp) - reference_stamp).seconds());
            if (delta_sec < best_delta_sec)
            {
                best_delta_sec = delta_sec;
                best_cloud = candidate;
            }
        }

        // 許容時刻差を超えるLiDARはscan欠落として扱い、基準LiDARだけで処理を継続する。
        if (best_cloud == nullptr || best_delta_sec > max_stamp_delta_sec_)
            return std::nullopt;
        return best_cloud;
    }

    std::size_t referenceQueueSize() const
    {
        const auto reference_iterator = queues_.find(reference_topic_);
        if (reference_iterator == queues_.end())
            return 0U;
        return reference_iterator->second.size();
    }

    void clearNonReferenceQueues()
    {
        // bag終端では非基準LiDARだけが残ることがあるため、基準scanなしならdrain可能な状態へ戻す。
        for (auto& [topic, queue] : queues_)
        {
            if (topic == reference_topic_)
                continue;
            queue.clear();
        }
    }

    void recordMatchStats(const MatchedLidarScanGroup& group)
    {
        matched_count_ += group.scans.size();
        if (group.scans.size() < specs_.size())
            missing_count_ += specs_.size() - group.scans.size();
    }

    void pruneProcessedGroup(const rclcpp::Time& processed_reference_stamp)
    {
        auto reference_iterator = queues_.find(reference_topic_);
        if (reference_iterator != queues_.end())
        {
            while (!reference_iterator->second.empty())
            {
                const auto stamp = rclcpp::Time(reference_iterator->second.front()->header.stamp);
                if (stamp > processed_reference_stamp)
                    break;
                reference_iterator->second.pop_front();
            }
        }

        // 非基準LiDARは次回の最近傍探索に必要な境界候補を残しつつ、確定済みstampを剪定する。
        for (auto& [topic, queue] : queues_)
        {
            if (topic == reference_topic_)
                continue;
            while (queue.size() >= 2U)
            {
                if (queue[1] == nullptr)
                {
                    queue.pop_front();
                    continue;
                }
                if (rclcpp::Time(queue[1]->header.stamp) > processed_reference_stamp)
                    break;
                queue.pop_front();
            }
            enforceQueueLimit(queue);
        }

        flush_oldest_reference_ = false;
    }

    void enforceQueueLimit(std::deque<CustomMsg::ConstSharedPtr>& queue) const
    {
        if (max_queue_size_ == 0U)
            return;
        while (queue.size() > max_queue_size_)
            queue.pop_front();
    }

    std::vector<InputLidarSpec> specs_;
    std::string reference_topic_;
    double max_stamp_delta_sec_;
    std::size_t max_queue_size_;
    std::unordered_map<std::string, std::deque<CustomMsg::ConstSharedPtr>> queues_;
    bool flush_oldest_reference_{false};
    unsigned long enqueued_count_{0UL};
    unsigned long processed_count_{0UL};
    unsigned long dropped_count_{0UL};
    unsigned long matched_count_{0UL};
    unsigned long missing_count_{0UL};
};

}  // namespace lio_sam
