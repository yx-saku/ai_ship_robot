#include "utility.hpp"
#include "lio_sam/msg/cloud_info.hpp"
#include "lio_sam/srv/save_map.hpp"
#include "lidar_transform_cache.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "multi_lidar_scan_synchronizer.hpp"

// AI_SHIP_ROBOT_BEGIN: LiDAR callbackを軽量化し、timerで複数scan処理と処理時間計測を行う。
#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <unordered_map>
// AI_SHIP_ROBOT_END

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct LiovxPointCustomMsg
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float time;
    uint16_t ring;
    uint16_t tag;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (LiovxPointCustomMsg,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (float, time, time)
    (uint16_t, ring, ring) (uint16_t, tag, tag)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = LiovxPointCustomMsg;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;
    // AI_SHIP_ROBOT_BEGIN: LiDAR queue操作とdeskew待機/破棄をtimer側へ集約する。
    std::mutex cloudLock;

    enum class DeskewStatus
    {
        Ready,
        Wait,
        Drop,
    };

    struct LidarReceiveStats
    {
        uint64_t count = 0;
        double firstStamp = 0.0;
        double lastStamp = 0.0;
        double gapSum = 0.0;
        double maxGap = 0.0;
        double lastLoggedStamp = 0.0;
        uint64_t lastLoggedCount = 0;
    };
    // AI_SHIP_ROBOT_END

    std::vector<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr> subLaserClouds;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
    // AI_SHIP_ROBOT_BEGIN: 点群変換をcallbackから切り離すため、専用callback groupとtimerを使う。
    rclcpp::CallbackGroup::SharedPtr callbackGroupCloudQueue;
    rclcpp::TimerBase::SharedPtr cloudQueueTimer;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    lio_sam::LidarTransformCache lidarTransformCache;
    std::unique_ptr<lio_sam::MultiLidarScanSynchronizer> multiLidarSynchronizer;
    // AI_SHIP_ROBOT_END
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<lio_sam::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;
    rclcpp::Service<lio_sam::srv::SaveMap>::SharedPtr srvCloudQueueEmpty;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;
    // AI_SHIP_ROBOT_BEGIN: 6軸IMU初期姿勢確定前のIMUをrawで保持し、確定後に再変換する。
    std::deque<sensor_msgs::msg::Imu> rawImuQueue;
    bool deferredRawImuConverted = false;
    bool cloudQueueHeldForInitialImuLogged = false;
    bool cloudQueueResumedAfterInitialImuLogged = false;
    // AI_SHIP_ROBOT_END

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    livox_ros_driver2::msg::CustomMsg currentCloudMsg;
    string lastDropReason = "none";
    std::chrono::steady_clock::time_point lastImageProjectionTimingLogTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastLidarReceiveStatsLogTime = std::chrono::steady_clock::now();
    std::unordered_map<std::string, LidarReceiveStats> lidarReceiveStatsByTopic;
    uint64_t imageProjectionTimingScanCount = 0;
    double imageProjectionCacheMsSum = 0.0;
    double imageProjectionDeskewMsSum = 0.0;
    double imageProjectionProjectMsSum = 0.0;
    double imageProjectionExtractionMsSum = 0.0;
    double imageProjectionPublishMsSum = 0.0;
    double imageProjectionTotalMsSum = 0.0;
    double rawNearVoxelMsSum = 0.0;
    size_t rawNearVoxelInputSum = 0;
    size_t rawNearVoxelOutputSum = 0;
    double lastRawNearVoxelMs = 0.0;
    size_t lastRawNearVoxelInputSize = 0;
    size_t lastRawNearVoxelOutputSize = 0;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;
    pcl::PointCloud<PointType>::Ptr   rawNearCloud;
    pcl::PointCloud<PointType>::Ptr   rawNearCloudDS;

    int deskewFlag;
    cv::Mat rangeMat;
    pcl::VoxelGrid<PointType> downSizeFilterRawNear;

    bool odomDeskewFlag;
    // AI_SHIP_ROBOT_BEGIN: odom補間deskewと初回scan publish待機を明示的な状態で管理する。
    bool odomStartAvailable;
    bool initialCloudWithoutOdomPublished = false;
    Eigen::Affine3f transStartOdom;
    // AI_SHIP_ROBOT_END
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::msg::CloudInfo cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::msg::Header cloudHeader;

    vector<int> columnIdnCountVec;


public:
    ImageProjection(const rclcpp::NodeOptions & options) :
            ParamServer("lio_sam_imageProjection", options),
            tfBuffer(get_clock()),
            tfListener(tfBuffer, this),
            deskewFlag(0)
    {
        callbackGroupLidar = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupImu = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupOdom = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupCloudQueue = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto lidarOpt = rclcpp::SubscriptionOptions();
        lidarOpt.callback_group = callbackGroupLidar;
        auto imuOpt = rclcpp::SubscriptionOptions();
        imuOpt.callback_group = callbackGroupImu;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = callbackGroupOdom;

        subImu = create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, qos_imu,
            std::bind(&ImageProjection::imuHandler, this, std::placeholders::_1),
            imuOpt);
        subOdom = create_subscription<nav_msgs::msg::Odometry>(
            odomTopic + "_incremental", qos_imu,
            std::bind(&ImageProjection::odometryHandler, this, std::placeholders::_1),
            odomOpt);
        validateMultiLidarInputParameters();
        multiLidarSynchronizer = std::make_unique<lio_sam::MultiLidarScanSynchronizer>(
            buildInputLidarSpecs(), referenceCustomTopic, maxStampDeltaSec, queueLength);
        subLaserClouds.reserve(inputCustomTopics.size());
        for (const auto& inputTopic : inputCustomTopics)
        {
            // 各LiDAR callbackはtopic名とshared_ptrを同期キューへ渡すだけにし、重い処理をtimerへ寄せる。
            subLaserClouds.push_back(create_subscription<livox_ros_driver2::msg::CustomMsg>(
                inputTopic, qos_lidar,
                [this, inputTopic](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr laserCloudMsg)
                {
                    cloudHandler(inputTopic, laserCloudMsg);
                },
                lidarOpt));
        }
        // AI_SHIP_ROBOT_BEGIN: 1回のtimerで1scanだけ処理し、後段queueのburstを避ける。
        cloudQueueTimer = create_wall_timer(
            std::chrono::milliseconds(5), std::bind(&ImageProjection::processCloudQueue, this), callbackGroupCloudQueue);
        // AI_SHIP_ROBOT_END

        if (publishDeskewedCloud)
        {
            // 診断用deskew点群は明示有効化された時だけpublisherを作り、通常運用のtopic数を増やさない。
            pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
                "lio_sam/deskew/cloud_deskewed", 1);
        }
        // rosbag再生や高頻度入力でCloudInfoが欠落しないよう、後段との接続だけreliable高depthにする。
        const auto cloudInfoQos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable().durability_volatile();
        pubLaserCloudInfo = create_publisher<lio_sam::msg::CloudInfo>(
            "lio_sam/deskew/cloud_info", cloudInfoQos);
        // rosbag再生後に未処理scanが残っていないかscript側から確認できるようにする。
        srvCloudQueueEmpty = create_service<lio_sam::srv::SaveMap>(
            "lio_sam/deskew/is_cloud_queue_empty",
            std::bind(&ImageProjection::cloudQueueEmptyHandler, this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

        RCLCPP_INFO(
            get_logger(),
            "Direct multi-LiDAR ImageProjection input: topics=%zu reference=%s max_stamp_delta=%.3f tf_timeout=%.3f",
            inputCustomTopics.size(), referenceCustomTopic.c_str(), maxStampDeltaSec, tfTimeoutSec);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        rawNearCloud.reset(new pcl::PointCloud<PointType>());
        rawNearCloudDS.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        downSizeFilterRawNear.setLeafSize(
            hybridRegisteredCloudRawNearLeafSize,
            hybridRegisteredCloudRawNearLeafSize,
            hybridRegisteredCloudRawNearLeafSize);

        cloudInfo.start_ring_index.assign(N_SCAN, 0);
        cloudInfo.end_ring_index.assign(N_SCAN, 0);

        cloudInfo.point_col_ind.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.point_range.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        rawNearCloud->clear();
        rawNearCloudDS->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;
        odomStartAvailable = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(N_SCAN, 0);
    }

    ~ImageProjection(){}

    void validateMultiLidarInputParameters() const
    {
        if (sensor != SensorType::LIVOX)
            throw std::runtime_error("multi-LiDAR direct imageProjection supports only sensor=livox.");
        if (inputCustomTopics.empty())
            throw std::runtime_error("input_custom_topics must not be empty for lio_sam_imageProjection.");
        if (referenceCustomTopic.empty())
            throw std::runtime_error("reference_custom_topic must not be empty for lio_sam_imageProjection.");
        if (std::find(inputCustomTopics.begin(), inputCustomTopics.end(), referenceCustomTopic) == inputCustomTopics.end())
            throw std::runtime_error("reference_custom_topic must be included in input_custom_topics.");
        if (!inputRingOffsets.empty() && inputRingOffsets.size() != inputCustomTopics.size())
            throw std::runtime_error("input_ring_offsets must be empty or match input_custom_topics size.");
        if (maxStampDeltaSec < 0.0 || tfTimeoutSec < 0.0 || timestampUnitScale <= 0.0)
            throw std::runtime_error("Invalid multi-LiDAR timing parameter.");

        for (std::size_t i = 0; i < inputCustomTopics.size(); ++i)
        {
            if (inputCustomTopics[i].empty())
                throw std::runtime_error("input_custom_topics must not contain an empty topic.");
            for (std::size_t j = i + 1; j < inputCustomTopics.size(); ++j)
            {
                if (inputCustomTopics[i] == inputCustomTopics[j])
                    throw std::runtime_error("input_custom_topics must not contain duplicate topics.");
            }
        }

        // ring offsetはLiDARごとの行帯域をずらすための値なので、開始行がN_SCAN外なら起動時に止める。
        for (const auto& ringOffset : inputRingOffsets)
        {
            if (ringOffset < 0 || ringOffset >= N_SCAN)
                throw std::runtime_error("input_ring_offsets must be in the range [0, N_SCAN).");
        }
    }

    std::vector<lio_sam::InputLidarSpec> buildInputLidarSpecs() const
    {
        std::vector<lio_sam::InputLidarSpec> specs;
        specs.reserve(inputCustomTopics.size());
        for (std::size_t i = 0; i < inputCustomTopics.size(); ++i)
        {
            const auto ringOffset = inputRingOffsets.empty() ? 0 : static_cast<int>(inputRingOffsets[i]);
            specs.push_back(lio_sam::InputLidarSpec{inputCustomTopics[i], ringOffset});
        }
        return specs;
    }

    void cloudQueueEmptyHandler(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<lio_sam::srv::SaveMap::Request> req,
        std::shared_ptr<lio_sam::srv::SaveMap::Response> res)
    {
        (void)request_header;
        (void)req;

        lio_sam::MultiLidarQueueStats stats;
        {
            std::lock_guard<std::mutex> lock(cloudLock);
            if (multiLidarSynchronizer != nullptr)
            {
                stats = multiLidarSynchronizer->stats();
                if (stats.queued_clouds > 0U)
                {
                    multiLidarSynchronizer->requestFlushOldestReference();
                    stats = multiLidarSynchronizer->stats();
                }
            }
        }

        // SaveMap service型をqueue空判定に流用し、bag終端の基準scanはflush要求で処理を進める。
        res->success = stats.queued_clouds == 0U;
        RCLCPP_INFO(
            get_logger(),
            "Cloud queue drain status: empty=%d queued_clouds=%zu reference_queue=%zu enqueued=%lu processed=%lu dropped=%lu matched=%lu missing=%lu",
            res->success ? 1 : 0, stats.queued_clouds, stats.reference_queue_size,
            stats.enqueued, stats.processed, stats.dropped, stats.matched, stats.missing);
    }

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
    {
        sensor_msgs::msg::Imu rawImu = *imuMsg;
        sensor_msgs::msg::Imu thisImu = imuConverter(rawImu);

        {
            std::lock_guard<std::mutex> lock1(imuLock);
            // 初期姿勢待機中のIMUは変換済みqueueへ入れず、姿勢確定後にraw履歴から作り直す。
            if (usingSixAxisImu() && waitForImuInitialization && !deferredRawImuConverted)
            {
                rawImuQueue.push_back(rawImu);
                if (!sixAxisImuReady())
                    return;

                imuQueue.clear();
                for (const auto& queuedRawImu : rawImuQueue)
                    imuQueue.push_back(imuConverter(queuedRawImu));
                RCLCPP_INFO(
                    get_logger(),
                    "Converted deferred raw IMU queue after 6-axis initialization: samples=%zu",
                    rawImuQueue.size());
                rawImuQueue.clear();
                deferredRawImuConverted = true;
                return;
            }

            imuQueue.push_back(thisImu);
        }

    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg)
    {
        {
            std::lock_guard<std::mutex> lock2(odoLock);
            odomQueue.push_back(*odometryMsg);
        }
    }

    // AI_SHIP_ROBOT_BEGIN: LiDAR callbackはenqueueだけを担当し、重い変換とdeskewはtimer側で行う。
    void cloudHandler(
        const std::string& topic,
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr laserCloudMsg)
    {
        {
            std::lock_guard<std::mutex> lock(cloudLock);
            if (multiLidarSynchronizer != nullptr)
                multiLidarSynchronizer->enqueue(topic, laserCloudMsg);
            recordLidarReceiveStatsLocked(topic, laserCloudMsg);
        }
    }

    void recordLidarReceiveStatsLocked(
        const std::string& topic,
        const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& laserCloudMsg)
    {
        if (laserCloudMsg == nullptr)
            return;

        // callback到達済みのCustomMsgだけをtopic別に数え、DDS/rosbag由来の受信欠落を処理段より前で切り分ける。
        auto& stats = lidarReceiveStatsByTopic[topic];
        const double stampSec = stamp2Sec(laserCloudMsg->header.stamp);
        if (stats.count == 0)
        {
            stats.firstStamp = stampSec;
            stats.lastStamp = stampSec;
            stats.lastLoggedStamp = stampSec;
        }
        else
        {
            const double gapSec = stampSec - stats.lastStamp;
            if (gapSec > 0.0)
            {
                stats.gapSum += gapSec;
                stats.maxGap = std::max(stats.maxGap, gapSec);
            }
            stats.lastStamp = stampSec;
        }
        ++stats.count;

        maybeLogLidarReceiveStatsLocked();
    }

    void maybeLogLidarReceiveStatsLocked()
    {
        if (processingTimeLogIntervalSec <= 0.0)
            return;

        const auto now = std::chrono::steady_clock::now();
        const double elapsedSec = std::chrono::duration<double>(now - lastLidarReceiveStatsLogTime).count();
        if (elapsedSec < processingTimeLogIntervalSec)
            return;

        // 各LiDAR topicの累計と直近区間を同時に出し、初期未接続と再生中dropを見分けられるようにする。
        for (const auto& inputTopic : inputCustomTopics)
        {
            const auto iterator = lidarReceiveStatsByTopic.find(inputTopic);
            if (iterator == lidarReceiveStatsByTopic.end())
            {
                RCLCPP_WARN(
                    get_logger(),
                    "LiDAR receive stats: topic=%s count=0 no messages reached imageProjection callback yet",
                    inputTopic.c_str());
                continue;
            }

            const auto& stats = iterator->second;
            const uint64_t intervalCount = stats.count - stats.lastLoggedCount;
            const double intervalStampSpan = stats.lastStamp - stats.lastLoggedStamp;
            const double totalStampSpan = stats.lastStamp - stats.firstStamp;
            const double avgGap = stats.count > 1 ? stats.gapSum / static_cast<double>(stats.count - 1) : 0.0;
            const double expectedCountAt10Hz = totalStampSpan > 0.0 ? totalStampSpan / 0.1 + 1.0 : static_cast<double>(stats.count);
            const double missingEstimateAt10Hz = std::max(0.0, expectedCountAt10Hz - static_cast<double>(stats.count));
            RCLCPP_INFO(
                get_logger(),
                "LiDAR receive stats: topic=%s count=%lu interval_count=%lu stamp=%.6f->%.6f interval_span=%.3f total_span=%.3f avg_gap=%.3f max_gap=%.3f expected_10hz=%.1f missing_est_10hz=%.1f",
                inputTopic.c_str(),
                static_cast<unsigned long>(stats.count),
                static_cast<unsigned long>(intervalCount),
                stats.firstStamp,
                stats.lastStamp,
                intervalStampSpan,
                totalStampSpan,
                avgGap,
                stats.maxGap,
                expectedCountAt10Hz,
                missingEstimateAt10Hz);
        }

        for (auto& [topic, stats] : lidarReceiveStatsByTopic)
        {
            stats.lastLoggedCount = stats.count;
            stats.lastLoggedStamp = stats.lastStamp;
        }
        lastLidarReceiveStatsLogTime = now;
    }

    bool shouldHoldCloudQueueForInitialImu()
    {
        std::lock_guard<std::mutex> lock(imuLock);
        return usingSixAxisImu() && waitForImuInitialization && !deferredRawImuConverted;
    }

    lio_sam::MultiLidarQueueStats currentCloudQueueStats()
    {
        std::lock_guard<std::mutex> lock(cloudLock);
        if (multiLidarSynchronizer == nullptr)
            return lio_sam::MultiLidarQueueStats();
        return multiLidarSynchronizer->stats();
    }

    bool holdCloudQueueUntilInitialImuReady()
    {
        if (!shouldHoldCloudQueueForInitialImu())
            return false;

        // LiDAR callbackは止めず、初期姿勢確定までscan処理だけ止めてqueue先頭から再開できるようにする。
        const auto stats = currentCloudQueueStats();
        if (!cloudQueueHeldForInitialImuLogged)
        {
            RCLCPP_INFO(
                get_logger(),
                "Hold LiDAR cloudQueue until IMU initialization completes: queued_clouds=%zu reference_queue=%zu enqueued=%lu",
                stats.queued_clouds, stats.reference_queue_size, stats.enqueued);
            cloudQueueHeldForInitialImuLogged = true;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Holding LiDAR cloudQueue until IMU initialization completes: queued_clouds=%zu reference_queue=%zu enqueued=%lu",
                stats.queued_clouds, stats.reference_queue_size, stats.enqueued);
        }
        return true;
    }

    void logCloudQueueResumeAfterInitialImu()
    {
        if (!cloudQueueHeldForInitialImuLogged || cloudQueueResumedAfterInitialImuLogged)
            return;

        // 初期化中に蓄積したLiDAR scanを、ここからtimer側で古い順に処理することを明示する。
        const auto stats = currentCloudQueueStats();
        RCLCPP_INFO(
            get_logger(),
            "Resume LiDAR cloudQueue after IMU initialization: queued_clouds=%zu reference_queue=%zu enqueued=%lu",
            stats.queued_clouds, stats.reference_queue_size, stats.enqueued);
        cloudQueueResumedAfterInitialImuLogged = true;
    }

    double elapsedMilliseconds(
        const std::chrono::steady_clock::time_point& start,
        const std::chrono::steady_clock::time_point& end) const
    {
        return std::chrono::duration<double, std::milli>(end - start).count();
    }

    void resetImageProjectionTimingAccumulator()
    {
        imageProjectionTimingScanCount = 0;
        imageProjectionCacheMsSum = 0.0;
        imageProjectionDeskewMsSum = 0.0;
        imageProjectionProjectMsSum = 0.0;
        imageProjectionExtractionMsSum = 0.0;
        imageProjectionPublishMsSum = 0.0;
        imageProjectionTotalMsSum = 0.0;
        rawNearVoxelMsSum = 0.0;
        rawNearVoxelInputSum = 0;
        rawNearVoxelOutputSum = 0;
    }

    void recordImageProjectionTiming(
        const double cacheMs,
        const double deskewMs,
        const double projectMs,
        const double extractionMs,
        const double publishMs,
        const double totalMs,
        const size_t queueStartSize,
        const size_t queueRemainingSize)
    {
        if (processingTimeLogIntervalSec <= 0.0)
            return;

        // 処理時間はscanごとの瞬間値ではなく平均で出し、負荷傾向とbacklogの増減を追いやすくする。
        ++imageProjectionTimingScanCount;
        imageProjectionCacheMsSum += cacheMs;
        imageProjectionDeskewMsSum += deskewMs;
        imageProjectionProjectMsSum += projectMs;
        imageProjectionExtractionMsSum += extractionMs;
        imageProjectionPublishMsSum += publishMs;
        imageProjectionTotalMsSum += totalMs;
        rawNearVoxelMsSum += lastRawNearVoxelMs;
        rawNearVoxelInputSum += lastRawNearVoxelInputSize;
        rawNearVoxelOutputSum += lastRawNearVoxelOutputSize;

        const auto now = std::chrono::steady_clock::now();
        const double elapsedSec = std::chrono::duration<double>(now - lastImageProjectionTimingLogTime).count();
        const bool backlogNeedsLog = queueStartSize >= static_cast<size_t>(imageProjectionBacklogLogThreshold) && elapsedSec >= 1.0;
        if (elapsedSec < processingTimeLogIntervalSec && !backlogNeedsLog)
            return;

        const double count = static_cast<double>(std::max<uint64_t>(imageProjectionTimingScanCount, 1));
        RCLCPP_INFO(
            get_logger(),
            "ImageProjection timing: scans=%lu avg_ms total=%.3f cache=%.3f deskew=%.3f project=%.3f extraction=%.3f publish=%.3f raw_near_voxel=%.3f raw_near_points=%.1f->%.1f queue=%zu->%zu max_per_timer=%d",
            static_cast<unsigned long>(imageProjectionTimingScanCount),
            imageProjectionTotalMsSum / count,
            imageProjectionCacheMsSum / count,
            imageProjectionDeskewMsSum / count,
            imageProjectionProjectMsSum / count,
            imageProjectionExtractionMsSum / count,
            imageProjectionPublishMsSum / count,
            rawNearVoxelMsSum / count,
            static_cast<double>(rawNearVoxelInputSum) / count,
            static_cast<double>(rawNearVoxelOutputSum) / count,
            queueStartSize,
            queueRemainingSize,
            imageProjectionMaxScansPerTimer);
        lastImageProjectionTimingLogTime = now;
        resetImageProjectionTimingAccumulator();
    }

    void processCloudQueue()
    {
        // backlog時は同じtimer callback内で複数scanを進め、再生後の未処理queue滞留を短縮する。
        int processedInThisTimer = 0;
        for (; processedInThisTimer < imageProjectionMaxScansPerTimer; ++processedInThisTimer)
        {
            if (!processOneCloudQueueScan())
                break;
        }

        size_t remainingCloudCount = 0;
        {
            std::lock_guard<std::mutex> lock(cloudLock);
            if (multiLidarSynchronizer != nullptr)
                remainingCloudCount = multiLidarSynchronizer->stats().queued_clouds;
        }
        if (processedInThisTimer >= imageProjectionMaxScansPerTimer && remainingCloudCount > 0)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "ImageProjection cloudQueue backlog remains after batch: processed_in_timer=%d remaining=%zu max_per_timer=%d",
                processedInThisTimer, remainingCloudCount, imageProjectionMaxScansPerTimer);
        }
    }

    bool processOneCloudQueueScan()
    {
        if (!rclcpp::ok())
            return false;
        if (holdCloudQueueUntilInitialImuReady())
            return false;
        logCloudQueueResumeAfterInitialImu();

        size_t queuedCloudCount = 0;
        lio_sam::MatchedLidarScanGroup scanGroup;
        const auto totalStartTime = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(cloudLock);
            if (multiLidarSynchronizer == nullptr || !multiLidarSynchronizer->buildNextGroup(&scanGroup))
                return false;
            queuedCloudCount = multiLidarSynchronizer->stats().queued_clouds;
        }

        const auto cacheStartTime = std::chrono::steady_clock::now();
        if (!cacheMultiLidarScanGroup(scanGroup))
        {
            // 変換不能scanを捨てる瞬間に、入力stampとqueue長を残してdrop原因を追跡できるようにする。
            RCLCPP_WARN(
                get_logger(),
                "Drop LiDAR scan before deskew: reason=%s stamp=%.6f matched_scans=%zu point_count=%zu queued_clouds=%zu",
                lastDropReason.c_str(), stamp2Sec(scanGroup.header.stamp),
                scanGroup.scans.size(), scanGroup.point_count, queuedCloudCount);
            // 空scanなど変換不能な入力はここで破棄し、queue操作の責務をtimer側へ集約する。
            std::lock_guard<std::mutex> lock(cloudLock);
            if (multiLidarSynchronizer != nullptr)
                multiLidarSynchronizer->markDropped(scanGroup);
            resetParameters();
            return true;
        }
        const auto cacheEndTime = std::chrono::steady_clock::now();

        const auto deskewStartTime = std::chrono::steady_clock::now();
        const DeskewStatus deskewStatus = deskewInfo();
        if (deskewStatus == DeskewStatus::Wait)
            return false;
        const auto deskewEndTime = std::chrono::steady_clock::now();
        if (deskewStatus == DeskewStatus::Drop)
        {
            size_t imuQueueSize = 0;
            size_t odomQueueSize = 0;
            {
                std::lock_guard<std::mutex> lock1(imuLock);
                imuQueueSize = imuQueue.size();
            }
            {
                std::lock_guard<std::mutex> lock2(odoLock);
                odomQueueSize = odomQueue.size();
            }
            // 復旧不能と判定したscanだけを捨て、原因と関連queue長を必ずログへ残す。
            RCLCPP_WARN(
                get_logger(),
                "Drop LiDAR scan during deskew: reason=%s scan_start=%.6f scan_end=%.6f queued_clouds=%zu imu_queue=%zu odom_queue=%zu",
                lastDropReason.c_str(), timeScanCur, timeScanEnd, queuedCloudCount,
                imuQueueSize, odomQueueSize);
            // 必要な時刻のIMUやodomが既に失われたscanだけを破棄し、後続scanの処理を止めない。
            std::lock_guard<std::mutex> lock(cloudLock);
            if (multiLidarSynchronizer != nullptr)
                multiLidarSynchronizer->markDropped(scanGroup);
            resetParameters();
            return true;
        }

        // 1回のtimerで1scanだけ処理し、後段featureExtraction/mapOptimizationの入力queueをburstで溢れさせない。
        const auto projectStartTime = std::chrono::steady_clock::now();
        projectMultiLidarCustomMsgs(scanGroup);
        const auto projectEndTime = std::chrono::steady_clock::now();
        const auto extractionStartTime = std::chrono::steady_clock::now();
        cloudExtraction();
        const auto extractionEndTime = std::chrono::steady_clock::now();
        const auto publishStartTime = std::chrono::steady_clock::now();
        publishClouds();
        const auto publishEndTime = std::chrono::steady_clock::now();

        size_t remainingCloudCount = 0;
        {
            std::lock_guard<std::mutex> lock(cloudLock);
            if (multiLidarSynchronizer != nullptr)
            {
                multiLidarSynchronizer->markProcessed(scanGroup);
                remainingCloudCount = multiLidarSynchronizer->stats().queued_clouds;
            }
        }
        recordImageProjectionTiming(
            elapsedMilliseconds(cacheStartTime, cacheEndTime),
            elapsedMilliseconds(deskewStartTime, deskewEndTime),
            elapsedMilliseconds(projectStartTime, projectEndTime),
            elapsedMilliseconds(extractionStartTime, extractionEndTime),
            elapsedMilliseconds(publishStartTime, publishEndTime),
            elapsedMilliseconds(totalStartTime, publishEndTime),
            queuedCloudCount,
            remainingCloudCount);
        resetParameters();
        return true;
    }
    // AI_SHIP_ROBOT_END

    double pointRelativeTimeSec(
        const lio_sam::MatchedLidarScan& scan,
        const livox_ros_driver2::msg::CustomPoint& point) const
    {
        return scan.stamp_delta_sec + static_cast<double>(point.offset_time) * timestampUnitScale;
    }

    bool transformLivoxPointToReference(
        const livox_ros_driver2::msg::CustomPoint& inputPoint,
        const lio_sam::RigidTransform& transform,
        PointType* outputPoint) const
    {
        if (outputPoint == nullptr)
            return false;

        // non-reference LiDARだけを基準LiDAR frameへ変換し、reference LiDARはidentityで分岐を最小化する。
        const float x = transform.identity ? inputPoint.x :
            transform.r00 * inputPoint.x + transform.r01 * inputPoint.y + transform.r02 * inputPoint.z + transform.tx;
        const float y = transform.identity ? inputPoint.y :
            transform.r10 * inputPoint.x + transform.r11 * inputPoint.y + transform.r12 * inputPoint.z + transform.ty;
        const float z = transform.identity ? inputPoint.z :
            transform.r20 * inputPoint.x + transform.r21 * inputPoint.y + transform.r22 * inputPoint.z + transform.tz;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            return false;

        outputPoint->x = x;
        outputPoint->y = y;
        outputPoint->z = z;
        outputPoint->intensity = inputPoint.reflectivity;
        return true;
    }

    bool cacheMultiLidarScanGroup(const lio_sam::MatchedLidarScanGroup& scanGroup)
    {
        cloudHeader = scanGroup.header;
        timeScanCur = stamp2Sec(cloudHeader.stamp);
        if (cloudHeader.frame_id.empty())
        {
            lastDropReason = "empty_reference_frame_id";
            return false;
        }
        if (scanGroup.point_count == 0U)
        {
            lastDropReason = "empty_multi_lidar_scan_group";
            RCLCPP_WARN(get_logger(), "Received an empty multi-LiDAR scan group; skip this scan.");
            return false;
        }

        double maxRelativeTimeSec = 0.0;
        bool unexpectedOffsetTime = false;
        for (const auto& scan : scanGroup.scans)
        {
            if (scan.cloud == nullptr)
                continue;
            for (const auto& point : scan.cloud->points)
            {
                const double relativeTimeSec = pointRelativeTimeSec(scan, point);
                if (!std::isfinite(relativeTimeSec))
                {
                    unexpectedOffsetTime = true;
                    continue;
                }
                if (relativeTimeSec < 0.0 || relativeTimeSec > maxPointOffsetTimeSec)
                    unexpectedOffsetTime = true;
                maxRelativeTimeSec = std::max(maxRelativeTimeSec, relativeTimeSec);
            }
        }
        timeScanEnd = timeScanCur + std::max(0.0, maxRelativeTimeSec);

        // LiDAR間stamp差を含めた相対時刻がdeskew範囲外なら、時刻設定の不整合をログで追えるようにする。
        if (unexpectedOffsetTime)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Unexpected multi-LiDAR relative point time: scan_start=%.6f scan_end=%.6f max_allowed=%.6f",
                timeScanCur, timeScanEnd, maxPointOffsetTimeSec);
        }

        return true;
    }

    bool shouldPrepareHybridRawNearCloud() const
    {
        return hybridRegisteredCloudEnabled;
    }

    void projectMultiLidarCustomMsgs(const lio_sam::MatchedLidarScanGroup& scanGroup)
    {
        const bool prepareHybridRawNearCloud = shouldPrepareHybridRawNearCloud();
        if (prepareHybridRawNearCloud)
        {
            // hybrid点群のraw詳細成分は点数が多いため、入力scan規模に合わせてcapacityを先に確保する。
            rawNearCloud->points.reserve(scanGroup.point_count);
        }
        for (const auto& scan : scanGroup.scans)
        {
            if (scan.cloud == nullptr)
                continue;
            if (!scan.is_reference && scan.cloud->header.frame_id.empty())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Skip non-reference LiDAR topic %s because frame_id is empty.",
                    scan.spec.topic.c_str());
                continue;
            }

            const auto transform = lidarTransformCache.lookup(
                scanGroup.header.frame_id, scan.cloud->header.frame_id, scan.cloud->header.stamp,
                tfTimeoutSec, tfBuffer, get_logger(), *get_clock());
            if (!transform.has_value())
            {
                if (scan.is_reference)
                    return;
                continue;
            }

            for (const auto& livoxPoint : scan.cloud->points)
            {
                const int rowIdn = static_cast<int>(livoxPoint.line) + scan.spec.ring_offset;
                if (rowIdn < 0 || rowIdn >= N_SCAN)
                    continue;

                PointType thisPoint;
                if (!transformLivoxPointToReference(livoxPoint, transform.value(), &thisPoint))
                    continue;

                const float range = pointDistance(thisPoint);
                if (range < lidarMinRange || range > lidarMaxRange)
                    continue;

                const double relativeTimeSec = pointRelativeTimeSec(scan, livoxPoint);
                bool pointDeskewedForRawNear = false;
                // hybrid点群のraw詳細成分はSLAM用downsampleとは独立にdeskew済みで保持する。
                if (prepareHybridRawNearCloud && range <= hybridRegisteredCloudRawNearRange)
                {
                    thisPoint = deskewPoint(&thisPoint, relativeTimeSec);
                    rawNearCloud->push_back(thisPoint);
                    pointDeskewedForRawNear = true;
                }

                if (rowIdn % downsampleRate != 0)
                    continue;

                int columnIdn = -1;
                if (sensor == SensorType::LIVOX)
                {
                    columnIdn = columnIdnCountVec[rowIdn];
                    columnIdnCountVec[rowIdn] += 1;
                }

                if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                    continue;
                if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                    continue;

                if (!pointDeskewedForRawNear)
                    thisPoint = deskewPoint(&thisPoint, relativeTimeSec);

                rangeMat.at<float>(rowIdn, columnIdn) = range;

                const int index = columnIdn + rowIdn * Horizon_SCAN;
                fullCloud->points[index] = thisPoint;
            }
        }
    }

    // 旧単一CustomMsg経路は直接multi-LiDAR投影へ置換済み。下記は未使用の既存変換処理として残す。

    // AI_SHIP_ROBOT_BEGIN: Livox CustomMsgを全点変換し、offset_time異常をdeskew品質の診断ログに出す。
    void moveFromCustomMsg(livox_ros_driver2::msg::CustomMsg &Msg, pcl::PointCloud<PointXYZIRT> & cloud)
    {
        cloud.clear();
        cloud.reserve(Msg.point_num);
        PointXYZIRT point;
        uint32_t minOffset = std::numeric_limits<uint32_t>::max();
        uint32_t maxOffset = 0;
        uint32_t lastOffset = 0;
        bool nonMonotonicOffset = false;

        cloud.header.frame_id=Msg.header.frame_id;
        cloud.header.stamp= (uint64_t)((Msg.header.stamp.sec*1e9 + Msg.header.stamp.nanosec)/1000) ;

        for(uint i=0;i<Msg.point_num;i++)
        {
            const uint32_t offset = Msg.points[i].offset_time;
            if (i > 0 && offset < lastOffset)
                nonMonotonicOffset = true;
            lastOffset = offset;
            minOffset = std::min(minOffset, offset);
            maxOffset = std::max(maxOffset, offset);

            point.x=Msg.points[i].x;
            point.y=Msg.points[i].y;
            point.z=Msg.points[i].z;
            point.intensity=Msg.points[i].reflectivity;
            point.tag=Msg.points[i].tag;
            point.time=offset*1e-9;
            point.ring=Msg.points[i].line;
            cloud.push_back(point);
        }

        // MID360のoffset_timeはdeskew品質へ直結するため、単位違いやsimulation由来の異常範囲をログで検出する。
        if (Msg.point_num > 0)
        {
            const double minOffsetSec = static_cast<double>(minOffset) * 1e-9;
            const double maxOffsetSec = static_cast<double>(maxOffset) * 1e-9;
            if (nonMonotonicOffset || maxOffsetSec < 0.0 || maxOffsetSec > maxPointOffsetTimeSec)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Unexpected Livox offset_time range: min=%.9f max=%.9f sec non_monotonic=%d",
                    minOffsetSec, maxOffsetSec, nonMonotonicOffset ? 1 : 0);
            }
        }
    }
    // AI_SHIP_ROBOT_END

    // AI_SHIP_ROBOT_BEGIN: queue先頭scanだけを変換し、破棄判断はprocessCloudQueueへ返す。
    bool cachePointCloud()
    {
        // processCloudQueueで取り出したscanだけを変換し、重い処理中のcallback取りこぼしを避ける。

        if (sensor == SensorType::LIVOX){
            moveFromCustomMsg(currentCloudMsg, *laserCloudIn);
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown sensor type: " << int(sensor));
            rclcpp::shutdown();
        }

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        if (laserCloudIn->empty())
        {
            lastDropReason = "empty_livox_custom_msg";
            RCLCPP_WARN(get_logger(), "Received an empty Livox CustomMsg; skip this scan.");
            return false;
        }

        timeScanCur = stamp2Sec(cloudHeader.stamp);
        double maxPointTime = 0.0;
        for (const auto& point : laserCloudIn->points)
            maxPointTime = std::max(maxPointTime, static_cast<double>(point.time));
        timeScanEnd = timeScanCur + maxPointTime;

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            RCLCPP_ERROR(get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
            rclcpp::shutdown();
        }

        return true;
    }
    // AI_SHIP_ROBOT_END

    // AI_SHIP_ROBOT_BEGIN: 必要なIMU/odomが未到着ならWait、復旧不能ならDropとしてqueue側へ返す。
    DeskewStatus deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        if (shouldWaitForSixAxisImuInitialization())
        {
            // 初期姿勢確定前のscanも保持し、raw IMU履歴を再変換してから先頭scanから順に処理する。
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for initial 6-axis IMU roll/pitch estimate; queued raw IMU samples=%zu ...",
                rawImuQueue.size());
            return DeskewStatus::Wait;
        }

        const bool needImuAngular = deskewMode == "imu_angular";
        const bool needStartImuForInitialPose = usingSixAxisImu() || useImuPreintegrationInitialGuess || needImuAngular;
        if (needStartImuForInitialPose)
        {
            // 初回keyframeの姿勢基準を揃えるため、scan開始以前のIMUがないLiDAR scanは処理しない。
            if (imuQueue.empty())
            {
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "Waiting for IMU data before scan publish: scan_start=%.6f",
                    timeScanCur);
                return DeskewStatus::Wait;
            }
            if (stamp2Sec(imuQueue.front().header.stamp) > timeScanCur)
            {
                lastDropReason = "start_imu_unavailable";
                RCLCPP_WARN(
                    get_logger(),
                    "Drop scan because required start IMU is unavailable: scan_start=%.6f first_imu=%.6f imu_queue=%zu",
                    timeScanCur, stamp2Sec(imuQueue.front().header.stamp), imuQueue.size());
                return DeskewStatus::Drop;
            }
        }
        if (needImuAngular)
        {
            if (imuQueue.empty() || stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd)
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for IMU data ...");
                return DeskewStatus::Wait;
            }
        }

        // deskewModeごとに必要な情報だけを要求し、odom補間deskewではIMU角速度依存を下げる。
        if (!imuQueue.empty())
            imuDeskewInfo();
        else
            cloudInfo.imu_available = false;

        odomDeskewInfo();

        if (deskewMode != "off" && useImuPreintegrationInitialGuess && !cloudInfo.odom_available)
        {
            if (!initialCloudWithoutOdomPublished)
            {
                initialCloudWithoutOdomPublished = true;
                RCLCPP_INFO(
                    get_logger(),
                    "Publish initial scan without IMU preintegration odometry to initialize mapping: scan_start=%.6f",
                    timeScanCur);
                return DeskewStatus::Ready;
            }

            if (odomQueue.empty() || stamp2Sec(odomQueue.back().header.stamp) < timeScanCur)
            {
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "Waiting for IMU preintegration odometry before scan publish: scan_start=%.6f",
                    timeScanCur);
                return DeskewStatus::Wait;
            }
            if (stamp2Sec(odomQueue.front().header.stamp) > timeScanCur)
            {
                lastDropReason = "start_odometry_unavailable";
                RCLCPP_WARN(
                    get_logger(),
                    "Drop scan because required start odometry is unavailable: scan_start=%.6f first_odom=%.6f odom_queue=%zu",
                    timeScanCur, stamp2Sec(odomQueue.front().header.stamp), odomQueue.size());
                return DeskewStatus::Drop;
            }

            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for interpolated IMU preintegration odometry before scan publish: scan_start=%.6f",
                timeScanCur);
            return DeskewStatus::Wait;
        }

        if (deskewMode == "odom_interpolation" && !odomStartAvailable)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Odometry interpolation is not available yet; publish scan without odom deskew.");
        }

        return DeskewStatus::Ready;
    }
    // AI_SHIP_ROBOT_END

    void imuDeskewInfo()
    {
        cloudInfo.imu_available = false;

        while (!imuQueue.empty())
        {
            if (stamp2Sec(imuQueue.front().header.stamp) < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = stamp2Sec(thisImuMsg.header.stamp);

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);
            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imu_available = true;
    }

    // AI_SHIP_ROBOT_BEGIN: IMU preintegration odometryをscan内時刻へ補間してdeskewと初期値に使う。
    bool interpolateOdom(double targetTime, nav_msgs::msg::Odometry *odomOut)
    {
        if (odomQueue.empty() || stamp2Sec(odomQueue.front().header.stamp) > targetTime ||
            stamp2Sec(odomQueue.back().header.stamp) < targetTime)
            return false;

        for (int i = 1; i < (int)odomQueue.size(); ++i)
        {
            const double backTime = stamp2Sec(odomQueue[i - 1].header.stamp);
            const double frontTime = stamp2Sec(odomQueue[i].header.stamp);
            if (frontTime < targetTime)
                continue;

            const nav_msgs::msg::Odometry& back = odomQueue[i - 1];
            const nav_msgs::msg::Odometry& front = odomQueue[i];
            const double duration = frontTime - backTime;
            const double ratio = duration <= 0.0 ? 0.0 : (targetTime - backTime) / duration;

            // 位置は線形補間、姿勢はslerpで補間し、scan内の任意時刻poseを作る。
            *odomOut = front;
            odomOut->pose.pose.position.x = back.pose.pose.position.x + ratio * (front.pose.pose.position.x - back.pose.pose.position.x);
            odomOut->pose.pose.position.y = back.pose.pose.position.y + ratio * (front.pose.pose.position.y - back.pose.pose.position.y);
            odomOut->pose.pose.position.z = back.pose.pose.position.z + ratio * (front.pose.pose.position.z - back.pose.pose.position.z);

            tf2::Quaternion qBack, qFront;
            tf2::fromMsg(back.pose.pose.orientation, qBack);
            tf2::fromMsg(front.pose.pose.orientation, qFront);
            tf2::Quaternion qInterpolated = qBack.slerp(qFront, ratio);
            qInterpolated.normalize();
            odomOut->pose.pose.orientation = tf2::toMsg(qInterpolated);
            return true;
        }

        return false;
    }

    Eigen::Affine3f odomMsgToAffine(const nav_msgs::msg::Odometry& odomMsg)
    {
        tf2::Quaternion orientation;
        tf2::fromMsg(odomMsg.pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(
            odomMsg.pose.pose.position.x, odomMsg.pose.pose.position.y, odomMsg.pose.pose.position.z,
            roll, pitch, yaw);
    }

    void odomDeskewInfo()
    {
        cloudInfo.odom_available = false;
        odomDeskewFlag = false;
        odomStartAvailable = false;

        while (!odomQueue.empty())
        {
            if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.2)
                odomQueue.pop_front();
            else
                break;
        }

        nav_msgs::msg::Odometry startOdomMsg;
        if (!interpolateOdom(timeScanCur, &startOdomMsg))
            return;

        tf2::Quaternion orientation;
        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // mapOptimizationの初期値にはscan開始時刻へ補間したodomを渡し、キュー選択による時刻ずれを減らす。
        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
        cloudInfo.initial_guess_roll = roll;
        cloudInfo.initial_guess_pitch = pitch;
        cloudInfo.initial_guess_yaw = yaw;
        cloudInfo.odom_available = true;
        odomStartAvailable = true;
        transStartOdom = odomMsgToAffine(startOdomMsg);

        nav_msgs::msg::Odometry endOdomMsg;
        if (!interpolateOdom(timeScanEnd, &endOdomMsg))
            return;

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBt = transStartOdom.inverse() * odomMsgToAffine(endOdomMsg);

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);
        odomDeskewFlag = true;
    }
    // AI_SHIP_ROBOT_END

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    // AI_SHIP_ROBOT_BEGIN: deskewModeに応じてodom補間deskew、IMU角速度deskew、無補正を切り替える。
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        if (cloudInfo.odom_available == false || odomDeskewFlag == false)
            return;

        const double scanDuration = timeScanEnd - timeScanCur;
        if (scanDuration <= 0.0)
            return;

        const float ratio = static_cast<float>(relTime / scanDuration);
        *posXCur = ratio * odomIncreX;
        *posYCur = ratio * odomIncreY;
        *posZCur = ratio * odomIncreZ;
    }

    PointType applyDeskewTransform(PointType *point, const Eigen::Affine3f& transBt)
    {
        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;
        return newPoint;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewMode == "off")
            return *point;

        const double pointTime = timeScanCur + relTime;

        if (deskewMode == "odom_interpolation")
        {
            nav_msgs::msg::Odometry pointOdomMsg;
            if (!odomStartAvailable || !interpolateOdom(pointTime, &pointOdomMsg))
                return *point;
            return applyDeskewTransform(point, transStartOdom.inverse() * odomMsgToAffine(pointOdomMsg));
        }

        if (deskewFlag == -1 || cloudInfo.imu_available == false)
            return *point;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // IMU角速度積分deskewではscan開始から各点時刻までの相対姿勢へ点を戻す。
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        return applyDeskewTransform(point, transStartInverse * transFinal);
    }
    // AI_SHIP_ROBOT_END

    void projectPointCloud()
    {
        const bool prepareHybridRawNearCloud = shouldPrepareHybridRawNearCloud();
        int cloudSize = laserCloudIn->points.size();
        if (prepareHybridRawNearCloud)
        {
            // hybrid点群のraw詳細成分は点数が多いため、入力scan規模に合わせてcapacityを先に確保する。
            rawNearCloud->points.reserve(cloudSize);
        }
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            bool pointDeskewedForRawNear = false;
            // downsampleRateでSLAM用点群を落としても、地形保存用のdeskew済みraw点は先に確保する。
            if (prepareHybridRawNearCloud && range <= hybridRegisteredCloudRawNearRange)
            {
                thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
                rawNearCloud->push_back(thisPoint);
                pointDeskewedForRawNear = true;
            }

            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER)
            {
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0/float(Horizon_SCAN);
                columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            }
            else if (sensor == SensorType::LIVOX)
            {
                columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn] += 1;
            }


            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            if (!pointDeskewedForRawNear)
                thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.start_ring_index[i] = count - 1 + 5;
            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.point_col_ind[count] = j;
                    // save range info
                    cloudInfo.point_range[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.end_ring_index[i] = count -1 - 5;
        }
    }

    void populateHybridRawNearCloudInfo()
    {
        // hybrid点群のraw詳細成分をCloudInfoに同梱し、後段でfeature点群と同一stampとして扱う。
        lastRawNearVoxelMs = 0.0;
        lastRawNearVoxelInputSize = rawNearCloud->size();
        lastRawNearVoxelOutputSize = rawNearCloud->size();
        cloudInfo.cloud_deskewed_raw_near = sensor_msgs::msg::PointCloud2();
        if (!shouldPrepareHybridRawNearCloud())
            return;

        pcl::PointCloud<PointType>::Ptr rawNearCloudToPublish = rawNearCloud;
        if (hybridRegisteredCloudRawNearLeafSize > 0.0 && rawNearCloud->size() > 1)
        {
            const auto voxelStartTime = std::chrono::steady_clock::now();
            rawNearCloudDS->clear();
            downSizeFilterRawNear.setInputCloud(rawNearCloud);
            downSizeFilterRawNear.filter(*rawNearCloudDS);
            const auto voxelEndTime = std::chrono::steady_clock::now();
            rawNearCloudToPublish = rawNearCloudDS;
            lastRawNearVoxelMs = elapsedMilliseconds(voxelStartTime, voxelEndTime);
            lastRawNearVoxelOutputSize = rawNearCloudDS->size();
        }

        pcl::toROSMsg(*rawNearCloudToPublish, cloudInfo.cloud_deskewed_raw_near);
        cloudInfo.cloud_deskewed_raw_near.header.stamp = cloudHeader.stamp;
        cloudInfo.cloud_deskewed_raw_near.header.frame_id = lidarFrame;
    }

    void publishClouds()
    {
        lastRawNearVoxelMs = 0.0;
        lastRawNearVoxelInputSize = rawNearCloud->size();
        lastRawNearVoxelOutputSize = rawNearCloud->size();
        cloudInfo.header = cloudHeader;
        // CloudInfo内部ではdeskew済み点群が必須だが、外部diagnostic topicは必要時だけpublishする。
        pcl::toROSMsg(*extractedCloud, cloudInfo.cloud_deskewed);
        cloudInfo.cloud_deskewed.header.stamp = cloudHeader.stamp;
        cloudInfo.cloud_deskewed.header.frame_id = lidarFrame;
        if (publishDeskewedCloud && pubExtractedCloud != nullptr && pubExtractedCloud->get_subscription_count() != 0)
            pubExtractedCloud->publish(cloudInfo.cloud_deskewed);
        // PCD map用hybrid点群のraw詳細成分もCloudInfoに入れ、feature点群と同じmessageで渡す。
        populateHybridRawNearCloudInfo();
        pubLaserCloudInfo->publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto IP = std::make_shared<ImageProjection>(options);
    exec.add_node(IP);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Image Projection Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
