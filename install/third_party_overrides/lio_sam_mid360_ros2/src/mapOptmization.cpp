#include "utility.hpp"
#include "lio_sam/msg/cloud_info.hpp"
#include "lio_sam/srv/save_map.hpp"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/time.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;


class mapOptimization : public ParamServer
{

public:

    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrameHybrid;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;
    std::chrono::steady_clock::time_point lastHybridCloudTimingLogTime = std::chrono::steady_clock::now();
    uint64_t hybridCloudTimingCount = 0;
    double hybridCloudFromRosMsSum = 0.0;
    double hybridCloudAppendMsSum = 0.0;
    double hybridCloudPublishMsSum = 0.0;
    double hybridCloudTotalMsSum = 0.0;
    size_t hybridCloudRawNearPointsSum = 0;
    size_t hybridCloudFeaturePointsSum = 0;
    size_t hybridCloudOutputPointsSum = 0;

    struct LocalizationVoxelKey
    {
        int64_t ix = 0;
        int64_t iy = 0;
        int64_t iz = 0;

        bool operator==(const LocalizationVoxelKey &other) const
        {
            return ix == other.ix && iy == other.iy && iz == other.iz;
        }
    };

    struct LocalizationVoxelKeyHash
    {
        size_t operator()(const LocalizationVoxelKey &key) const
        {
            const auto hx = std::hash<int64_t>{}(key.ix);
            const auto hy = std::hash<int64_t>{}(key.iy);
            const auto hz = std::hash<int64_t>{}(key.iz);
            return hx ^ (hy << 1U) ^ (hz << 2U);
        }
    };

    struct LocalizationVoxelStats
    {
        size_t count = 0;
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        double sum_intensity = 0.0;

        void add(const Eigen::Vector3f &point, float intensity)
        {
            ++count;
            sum_x += point.x();
            sum_y += point.y();
            sum_z += point.z();
            sum_intensity += intensity;
        }

        void addWeighted(const Eigen::Vector3f &point, double intensitySum, size_t weight)
        {
            if (weight == 0)
                return;
            count += weight;
            sum_x += static_cast<double>(point.x()) * static_cast<double>(weight);
            sum_y += static_cast<double>(point.y()) * static_cast<double>(weight);
            sum_z += static_cast<double>(point.z()) * static_cast<double>(weight);
            sum_intensity += intensitySum;
        }

        void merge(const LocalizationVoxelStats &other)
        {
            count += other.count;
            sum_x += other.sum_x;
            sum_y += other.sum_y;
            sum_z += other.sum_z;
            sum_intensity += other.sum_intensity;
        }

        PointType centroidPoint() const
        {
            PointType point;
            point.x = 0.0f;
            point.y = 0.0f;
            point.z = 0.0f;
            point.intensity = 0.0f;
            if (count == 0)
                return point;
            const double denominator = static_cast<double>(count);
            point.x = static_cast<float>(sum_x / denominator);
            point.y = static_cast<float>(sum_y / denominator);
            point.z = static_cast<float>(sum_z / denominator);
            point.intensity = static_cast<float>(sum_intensity / denominator);
            return point;
        }
    };

    struct ElevationGridKey
    {
        int64_t ix = 0;
        int64_t iy = 0;

        bool operator==(const ElevationGridKey &other) const
        {
            return ix == other.ix && iy == other.iy;
        }
    };

    struct ElevationGridKeyHash
    {
        size_t operator()(const ElevationGridKey &key) const
        {
            const auto hx = std::hash<int64_t>{}(key.ix);
            const auto hy = std::hash<int64_t>{}(key.iy);
            return hx ^ (hy << 1U);
        }
    };

    struct ElevationClusterNode
    {
        ElevationGridKey key;
        size_t cluster_index = 0;

        bool operator==(const ElevationClusterNode &other) const
        {
            return key == other.key && cluster_index == other.cluster_index;
        }
    };

    struct ElevationClusterNodeHash
    {
        size_t operator()(const ElevationClusterNode &node) const
        {
            const auto keyHash = ElevationGridKeyHash{}(node.key);
            const auto indexHash = std::hash<size_t>{}(node.cluster_index);
            return keyHash ^ (indexHash << 1U);
        }
    };

    struct ElevationRunningStats
    {
        size_t count = 0;
        double z_min = std::numeric_limits<double>::infinity();
        double z_max = -std::numeric_limits<double>::infinity();
        double z_mean = 0.0;
        double z_m2 = 0.0;

        void add(double z)
        {
            ++count;
            z_min = std::min(z_min, z);
            z_max = std::max(z_max, z);
            const double delta = z - z_mean;
            z_mean += delta / static_cast<double>(count);
            const double delta_after = z - z_mean;
            z_m2 += delta * delta_after;
        }

        void merge(size_t other_count, double other_min, double other_max, double other_mean, double other_m2)
        {
            if (other_count == 0)
                return;
            if (count == 0)
            {
                count = other_count;
                z_min = other_min;
                z_max = other_max;
                z_mean = other_mean;
                z_m2 = other_m2;
                return;
            }

            const double total_count = static_cast<double>(count + other_count);
            const double delta = other_mean - z_mean;
            z_m2 += other_m2 + delta * delta * static_cast<double>(count) * static_cast<double>(other_count) / total_count;
            z_mean += delta * static_cast<double>(other_count) / total_count;
            z_min = std::min(z_min, other_min);
            z_max = std::max(z_max, other_max);
            count += other_count;
        }
    };

    struct ElevationLocalPoint
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct ElevationRawClusterCell
    {
        std::vector<ElevationLocalPoint> points;
        std::vector<double> z_values;
    };

    struct ElevationSubmapCell
    {
        int64_t ix = 0;
        int64_t iy = 0;
        double x = 0.0;
        double y = 0.0;
        std::vector<ElevationRunningStats> z_clusters;
    };

    struct KeyframeMapSubmap
    {
        size_t keyframe_index = 0;
        double keyframe_time = 0.0;
        PointTypePose initial_keyframe_pose;
        std::unordered_map<LocalizationVoxelKey, LocalizationVoxelStats, LocalizationVoxelKeyHash> localization_voxels;
        std::unordered_map<ElevationGridKey, ElevationSubmapCell, ElevationGridKeyHash> elevation_cells;
        size_t scan_count = 0;
        size_t raw_point_count = 0;
        size_t localization_point_count = 0;
        size_t elevation_input_point_count = 0;
    };

    std::vector<KeyframeMapSubmap> keyframeMapSubmaps;
    size_t keyframeSubmapProcessedScanCount = 0;
    size_t keyframeSubmapSkippedEmptyScanCount = 0;
    size_t keyframeSubmapTotalRawPointCount = 0;

    rclcpp::Service<lio_sam::srv::SaveMap>::SharedPtr srvSaveMap;
    rclcpp::Subscription<lio_sam::msg::CloudInfo>::SharedPtr subCloud;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGPS;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;

    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    lio_sam::msg::CloudInfo cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    rclcpp::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    Eigen::Matrix<float, 6, 6> matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer; // from new to old
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    deque<std_msgs::msg::Float64MultiArray> loopInfoVec;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    mapOptimization(const rclcpp::NodeOptions & options) :
        ParamServer("lio_sam_mapOptimization", options),
        tfBuffer(get_clock()),
        tfListener(tfBuffer, this)
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        if (publishTrajectoryCloud)
            pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/trajectory", 1);
        if (publishMapGlobalCloud)
            pubLaserCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>("lio_sam/mapping/odometry", qos);
        pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>(
            "lio_sam/mapping/odometry_incremental", qos);
        pubPath = create_publisher<nav_msgs::msg::Path>("lio_sam/mapping/path", 1);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // AI_SHIP_ROBOT_BEGIN: featureExtractionからmappingへCloudInfoを落とさないQoSにする。
        // rosbag再生や高頻度入力でCloudInfoが欠落しないよう、後段との接続だけreliable高depthにする。
        const auto cloudInfoQos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable().durability_volatile();
        subCloud = create_subscription<lio_sam::msg::CloudInfo>(
            "lio_sam/feature/cloud_info", cloudInfoQos,
            std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
        // AI_SHIP_ROBOT_END
        subGPS = create_subscription<nav_msgs::msg::Odometry>(
            gpsTopic, 200,
            std::bind(&mapOptimization::gpsHandler, this, std::placeholders::_1));
        subLoop = create_subscription<std_msgs::msg::Float64MultiArray>(
            "lio_loop/loop_closure_detection", qos,
            std::bind(&mapOptimization::loopInfoHandler, this, std::placeholders::_1));

        auto saveMapService = [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<lio_sam::srv::SaveMap::Request> req, std::shared_ptr<lio_sam::srv::SaveMap::Response> res) -> void {
            (void)request_header;
            res->success = saveMapToFiles(req);
        };

        srvSaveMap = create_service<lio_sam::srv::SaveMap>("lio_sam/save_map", saveMapService);
        if (publishLoopClosureClouds)
        {
            // loop closure可視化topicは通常運用では不要なため、診断時だけpublisherを作る。
            pubHistoryKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
            pubIcpKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
            pubLoopConstraintEdge = create_publisher<visualization_msgs::msg::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);
        }

        if (publishMapLocalCloud)
            pubRecentKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/map_local", 1);
        if (publishCloudRegistered)
            pubRecentKeyFrame = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
        if (publishCloudRegisteredRaw)
            pubCloudRegisteredRaw = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);
        if (hybridRegisteredCloudEnabled)
            pubRecentKeyFrameHybrid = create_publisher<sensor_msgs::msg::PointCloud2>(hybridRegisteredCloudTopic, 1);

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        allocateMemory();
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        matP.setZero();
    }

    std::filesystem::path resolveSaveMapDirectory(const std::string &destination) const
    {
        const char *home = std::getenv("HOME");
        const std::string homeDirectory = home == nullptr ? std::string() : std::string(home);
        if (destination.empty())
            return std::filesystem::path(homeDirectory + savePCDDirectory);

        const std::filesystem::path requestedPath(destination);
        if (requestedPath.is_absolute())
            return requestedPath;
        return std::filesystem::path(homeDirectory + destination);
    }

    std::vector<ElevationRunningStats> normalizeElevationClusters(std::vector<ElevationRunningStats> clusters) const
    {
        clusters.erase(
            std::remove_if(clusters.begin(), clusters.end(), [](const auto &cluster) { return cluster.count == 0; }),
            clusters.end());
        if (clusters.empty())
            return clusters;

        std::sort(clusters.begin(), clusters.end(), [](const auto &lhs, const auto &rhs) {
            if (lhs.z_min != rhs.z_min)
                return lhs.z_min < rhs.z_min;
            return lhs.z_max < rhs.z_max;
        });

        std::vector<ElevationRunningStats> mergedClusters;
        mergedClusters.reserve(clusters.size());
        for (const auto &cluster : clusters)
        {
            if (mergedClusters.empty() || cluster.z_min - mergedClusters.back().z_max > elevationCellZClusterGap)
            {
                mergedClusters.push_back(cluster);
                continue;
            }
            auto &target = mergedClusters.back();
            target.merge(cluster.count, cluster.z_min, cluster.z_max, cluster.z_mean, cluster.z_m2);
        }
        return mergedClusters;
    }

    std::vector<ElevationRunningStats> buildElevationZClusters(std::vector<double> values) const
    {
        std::vector<ElevationRunningStats> clusters;
        if (values.empty())
            return clusters;

        std::sort(values.begin(), values.end());
        size_t startIndex = 0;
        while (startIndex < values.size())
        {
            size_t endIndex = startIndex + 1;
            while (endIndex < values.size() && values[endIndex] - values[endIndex - 1] <= elevationCellZClusterGap)
                ++endIndex;

            ElevationRunningStats stats;
            for (size_t i = startIndex; i < endIndex; ++i)
                stats.add(values[i]);
            clusters.push_back(stats);
            startIndex = endIndex;
        }
        return normalizeElevationClusters(std::move(clusters));
    }

    size_t findElevationClusterIndex(const std::vector<ElevationRunningStats> &clusters, double z) const
    {
        size_t bestIndex = 0;
        double bestDistance = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            const auto &cluster = clusters[i];
            if (z >= cluster.z_min - 1.0e-9 && z <= cluster.z_max + 1.0e-9)
                return i;
            const double distance = std::min(std::abs(z - cluster.z_min), std::abs(z - cluster.z_max));
            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    bool elevationClusterOverlaps(const ElevationRunningStats &lhs, const ElevationRunningStats &rhs) const
    {
        if (lhs.count == 0 || rhs.count == 0)
            return false;
        return lhs.z_min <= rhs.z_max + elevationCellZClusterGap && rhs.z_min <= lhs.z_max + elevationCellZClusterGap;
    }

    double elevationClusterCenterX(const ElevationGridKey &key) const
    {
        return (static_cast<double>(key.ix) + 0.5) * elevationClusterCellSize;
    }

    double elevationClusterCenterY(const ElevationGridKey &key) const
    {
        return (static_cast<double>(key.iy) + 0.5) * elevationClusterCellSize;
    }

    double elevationClusterXyDistanceSquared(
        const ElevationGridKey &lhsKey,
        const ElevationGridKey &rhsKey) const
    {
        const double dx = elevationClusterCenterX(lhsKey) - elevationClusterCenterX(rhsKey);
        const double dy = elevationClusterCenterY(lhsKey) - elevationClusterCenterY(rhsKey);
        return dx * dx + dy * dy;
    }

    bool isBetterSelectedElevationCluster(
        const ElevationRunningStats &candidate,
        const ElevationRunningStats &current,
        double seedZ) const
    {
        const double candidateDistance = std::abs(candidate.z_mean - seedZ);
        const double currentDistance = std::abs(current.z_mean - seedZ);
        if (candidateDistance != currentDistance)
            return candidateDistance < currentDistance;
        if (candidate.count != current.count)
            return candidate.count > current.count;
        return candidate.z_mean < current.z_mean;
    }

    std::optional<ElevationClusterNode> findOriginNearestElevationCluster(
        const std::unordered_map<ElevationGridKey, std::vector<ElevationRunningStats>, ElevationGridKeyHash> &clusterCells) const
    {
        std::optional<ElevationClusterNode> bestNode;
        double bestDistance = std::numeric_limits<double>::infinity();
        for (const auto &entry : clusterCells)
        {
            const auto &key = entry.first;
            const auto &clusters = entry.second;
            for (size_t index = 0; index < clusters.size(); ++index)
            {
                const auto &cluster = clusters[index];
                if (cluster.count == 0)
                    continue;
                const double x = elevationClusterCenterX(key);
                const double y = elevationClusterCenterY(key);
                const double distance = x * x + y * y + cluster.z_mean * cluster.z_mean;
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestNode = ElevationClusterNode{key, index};
                }
            }
        }
        return bestNode;
    }

    std::unordered_map<ElevationGridKey, size_t, ElevationGridKeyHash> selectOriginConnectedElevationClusters(
        const std::unordered_map<ElevationGridKey, std::vector<ElevationRunningStats>, ElevationGridKeyHash> &clusterCells) const
    {
        std::unordered_map<ElevationGridKey, size_t, ElevationGridKeyHash> selected;
        const auto seedNode = findOriginNearestElevationCluster(clusterCells);
        if (!seedNode.has_value())
            return selected;

        const auto seedCellIter = clusterCells.find(seedNode->key);
        if (seedCellIter == clusterCells.end() || seedNode->cluster_index >= seedCellIter->second.size())
            return selected;
        const double seedZ = seedCellIter->second[seedNode->cluster_index].z_mean;
        const double connectionRadiusSquared = elevationClusterConnectionRadius * elevationClusterConnectionRadius;
        const int64_t radiusCells = static_cast<int64_t>(std::ceil(elevationClusterConnectionRadius / elevationClusterCellSize));
        std::vector<ElevationGridKey> neighborOffsets;
        for (int64_t dx = -radiusCells; dx <= radiusCells; ++dx)
        {
            for (int64_t dy = -radiusCells; dy <= radiusCells; ++dy)
                neighborOffsets.push_back(ElevationGridKey{dx, dy});
        }

        std::queue<ElevationClusterNode> queue;
        std::unordered_set<ElevationClusterNode, ElevationClusterNodeHash> visited;
        std::vector<ElevationClusterNode> connectedNodes;
        queue.push(*seedNode);
        visited.insert(*seedNode);

        // 原点最近傍clusterからXY距離と高さ差で到達可能なclusterだけを辿り、天井などの孤立成分を除外する。
        while (!queue.empty())
        {
            const auto node = queue.front();
            queue.pop();
            connectedNodes.push_back(node);

            const auto currentCellIter = clusterCells.find(node.key);
            if (currentCellIter == clusterCells.end() || node.cluster_index >= currentCellIter->second.size())
                continue;
            const auto &currentCluster = currentCellIter->second[node.cluster_index];

            for (const auto &offset : neighborOffsets)
            {
                const ElevationGridKey neighborKey{node.key.ix + offset.ix, node.key.iy + offset.iy};
                const auto neighborCellIter = clusterCells.find(neighborKey);
                if (neighborCellIter == clusterCells.end())
                    continue;

                const auto &neighborClusters = neighborCellIter->second;
                for (size_t neighborIndex = 0; neighborIndex < neighborClusters.size(); ++neighborIndex)
                {
                    const ElevationClusterNode neighborNode{neighborKey, neighborIndex};
                    if (visited.find(neighborNode) != visited.end())
                        continue;
                    if (elevationClusterXyDistanceSquared(node.key, neighborKey) > connectionRadiusSquared)
                        continue;
                    if (std::abs(currentCluster.z_mean - neighborClusters[neighborIndex].z_mean) > elevationClusterConnectionZGap)
                        continue;
                    visited.insert(neighborNode);
                    queue.push(neighborNode);
                }
            }
        }

        selected.reserve(connectedNodes.size());
        for (const auto &node : connectedNodes)
        {
            const auto cellIter = clusterCells.find(node.key);
            if (cellIter == clusterCells.end() || node.cluster_index >= cellIter->second.size())
                continue;
            const auto selectedIter = selected.find(node.key);
            if (selectedIter == selected.end())
            {
                selected.emplace(node.key, node.cluster_index);
                continue;
            }

            // 同じ30cm cellで複数clusterが到達した場合は、seedの高さに近いclusterだけを残す。
            const auto &candidate = cellIter->second[node.cluster_index];
            const auto &current = cellIter->second[selectedIter->second];
            if (isBetterSelectedElevationCluster(candidate, current, seedZ))
                selectedIter->second = node.cluster_index;
        }

        RCLCPP_INFO(
            get_logger(),
            "Origin-connected elevation clusters: seed=(%ld,%ld,%zu) seed_z=%.3f connection_radius=%.3f connection_z_gap=%.3f connected_clusters=%zu selected_cluster_cells=%zu",
            seedNode->key.ix, seedNode->key.iy, seedNode->cluster_index, seedZ,
            elevationClusterConnectionRadius, elevationClusterConnectionZGap, connectedNodes.size(), selected.size());
        return selected;
    }

    void mergeElevationClusters(
        std::vector<ElevationRunningStats> &target,
        const std::vector<ElevationRunningStats> &source) const
    {
        if (source.empty())
            return;
        target.insert(target.end(), source.begin(), source.end());
        target = normalizeElevationClusters(std::move(target));
    }

    void mergeElevationCell(ElevationSubmapCell &target, const ElevationSubmapCell &source) const
    {
        mergeElevationClusters(target.z_clusters, source.z_clusters);
    }

    ElevationGridKey elevationOutputKeyForPoint(const ElevationLocalPoint &point) const
    {
        return ElevationGridKey{
            static_cast<int64_t>(std::floor(point.x / elevationOutputCellSize)),
            static_cast<int64_t>(std::floor(point.y / elevationOutputCellSize))};
    }

    ElevationGridKey elevationClusterKeyForPoint(const ElevationLocalPoint &point) const
    {
        return ElevationGridKey{
            static_cast<int64_t>(std::floor(point.x / elevationClusterCellSize)),
            static_cast<int64_t>(std::floor(point.y / elevationClusterCellSize))};
    }

    std::vector<ElevationSubmapCell> extractElevationCells(const std::vector<ElevationLocalPoint> &points) const
    {
        std::unordered_map<ElevationGridKey, ElevationRawClusterCell, ElevationGridKeyHash> rawClusterCells;
        rawClusterCells.reserve(points.size());

        // z-layer判定は粗いcluster cellで行い、最終出力用の細かいcellへ点統計だけを振り分ける。
        for (const auto &point : points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;
            auto &clusterCell = rawClusterCells[elevationClusterKeyForPoint(point)];
            clusterCell.points.push_back(point);
            clusterCell.z_values.push_back(point.z);
        }

        std::unordered_map<ElevationGridKey, ElevationSubmapCell, ElevationGridKeyHash> outputCells;
        outputCells.reserve(points.size());
        for (auto &entry : rawClusterCells)
        {
            auto &rawCell = entry.second;
            auto clusters = buildElevationZClusters(std::move(rawCell.z_values));
            if (clusters.empty())
                continue;

            for (const auto &point : rawCell.points)
            {
                const auto outputKey = elevationOutputKeyForPoint(point);
                auto &outputCell = outputCells[outputKey];
                outputCell.ix = outputKey.ix;
                outputCell.iy = outputKey.iy;
                outputCell.x = (static_cast<double>(outputKey.ix) + 0.5) * elevationOutputCellSize;
                outputCell.y = (static_cast<double>(outputKey.iy) + 0.5) * elevationOutputCellSize;

                const size_t clusterIndex = findElevationClusterIndex(clusters, point.z);
                if (outputCell.z_clusters.size() <= clusterIndex)
                    outputCell.z_clusters.resize(clusterIndex + 1U);
                outputCell.z_clusters[clusterIndex].add(point.z);
            }
        }

        std::vector<ElevationSubmapCell> cells;
        cells.reserve(outputCells.size());
        for (auto &entry : outputCells)
        {
            auto &cell = entry.second;
            cell.z_clusters = normalizeElevationClusters(std::move(cell.z_clusters));
            if (!cell.z_clusters.empty())
                cells.push_back(std::move(cell));
        }
        std::sort(cells.begin(), cells.end(), [](const auto &lhs, const auto &rhs) {
            if (lhs.ix != rhs.ix)
                return lhs.ix < rhs.ix;
            return lhs.iy < rhs.iy;
        });
        return cells;
    }

    LocalizationVoxelKey localizationVoxelKeyForPoint(const Eigen::Vector3f &point) const
    {
        return LocalizationVoxelKey{
            static_cast<int64_t>(std::floor(static_cast<double>(point.x()) / localizationSubmapLeafSize)),
            static_cast<int64_t>(std::floor(static_cast<double>(point.y()) / localizationSubmapLeafSize)),
            static_cast<int64_t>(std::floor(static_cast<double>(point.z()) / localizationSubmapLeafSize))};
    }

    void ensureKeyframeMapSubmap(size_t keyframeIndex)
    {
        while (keyframeMapSubmaps.size() <= keyframeIndex)
        {
            const size_t index = keyframeMapSubmaps.size();
            KeyframeMapSubmap submap;
            submap.keyframe_index = index;
            if (index < cloudKeyPoses6D->size())
            {
                submap.keyframe_time = cloudKeyPoses6D->points[index].time;
                submap.initial_keyframe_pose = cloudKeyPoses6D->points[index];
            }
            keyframeMapSubmaps.push_back(std::move(submap));
        }
    }

    void accumulateKeyframeRawSubmap(
        size_t keyframeCountBeforeSave,
        size_t keyframeCountAfterSave,
        const PointTypePose &scanPose)
    {
        if (!saveElevationMap)
            return;
        if (keyframeCountAfterSave == 0U)
            return;

        ++keyframeSubmapProcessedScanCount;
        const bool assignedToNewKeyframe = keyframeCountAfterSave > keyframeCountBeforeSave;
        const size_t keyframeIndex = keyframeCountAfterSave - 1U;
        ensureKeyframeMapSubmap(keyframeIndex);

        if (static_cast<uint64_t>(cloudInfo.cloud_deskewed.width) * cloudInfo.cloud_deskewed.height == 0U)
        {
            ++keyframeSubmapSkippedEmptyScanCount;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Skip keyframe raw submap accumulation because cloud_deskewed is empty: processed_scans=%zu skipped_empty=%zu",
                keyframeSubmapProcessedScanCount, keyframeSubmapSkippedEmptyScanCount);
            return;
        }

        pcl::PointCloud<PointType> deskewedCloud;
        pcl::fromROSMsg(cloudInfo.cloud_deskewed, deskewedCloud);
        if (deskewedCloud.empty())
        {
            ++keyframeSubmapSkippedEmptyScanCount;
            return;
        }

        KeyframeMapSubmap &submap = keyframeMapSubmaps[keyframeIndex];
        const PointTypePose keyframePose = submap.initial_keyframe_pose;
        const Eigen::Affine3f mapFromScan = pclPointToAffine3f(scanPose);
        const Eigen::Affine3f mapFromKeyframe = pclPointToAffine3f(keyframePose);
        const Eigen::Affine3f keyframeFromScan = mapFromKeyframe.inverse() * mapFromScan;
        const double cosYaw = std::cos(keyframePose.yaw);
        const double sinYaw = std::sin(keyframePose.yaw);
        std::vector<ElevationLocalPoint> elevationPoints;
        elevationPoints.reserve(deskewedCloud.size());
        size_t finiteRawPoints = 0;
        size_t localizationPoints = 0;
        size_t elevationInputPoints = 0;

        // 1 scanのraw点を、localizationはkeyframe local XYZ、elevationは重力整列keyframe XY/Zへ同時に集約する。
        for (const auto &point : deskewedCloud.points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                continue;
            ++finiteRawPoints;

            const Eigen::Vector3f scanPoint(point.x, point.y, point.z);
            const Eigen::Vector3f keyframePoint = keyframeFromScan * scanPoint;
            if (std::isfinite(keyframePoint.x()) && std::isfinite(keyframePoint.y()) && std::isfinite(keyframePoint.z()))
            {
                const float intensity = std::isfinite(point.intensity) ? point.intensity : 0.0f;
                submap.localization_voxels[localizationVoxelKeyForPoint(keyframePoint)].add(keyframePoint, intensity);
                ++localizationPoints;
            }

            const double range = std::sqrt(
                static_cast<double>(point.x) * point.x + static_cast<double>(point.y) * point.y + static_cast<double>(point.z) * point.z);
            if (range < elevationMinRange || range > elevationMaxRange)
                continue;

            const Eigen::Vector3f mapPoint = mapFromScan * scanPoint;
            const double dx = static_cast<double>(mapPoint.x()) - keyframePose.x;
            const double dy = static_cast<double>(mapPoint.y()) - keyframePose.y;
            const double localX = cosYaw * dx + sinYaw * dy;
            const double localY = -sinYaw * dx + cosYaw * dy;
            const double localZ = static_cast<double>(mapPoint.z()) - keyframePose.z;
            if (!std::isfinite(localX) || !std::isfinite(localY) || !std::isfinite(localZ))
                continue;
            elevationPoints.push_back(ElevationLocalPoint{localX, localY, localZ});
            ++elevationInputPoints;
        }

        auto elevationCells = extractElevationCells(elevationPoints);
        for (const auto &cell : elevationCells)
        {
            const ElevationGridKey key{cell.ix, cell.iy};
            auto &targetCell = submap.elevation_cells[key];
            targetCell.ix = cell.ix;
            targetCell.iy = cell.iy;
            targetCell.x = cell.x;
            targetCell.y = cell.y;
            mergeElevationCell(targetCell, cell);
        }

        ++submap.scan_count;
        submap.raw_point_count += finiteRawPoints;
        submap.localization_point_count += localizationPoints;
        submap.elevation_input_point_count += elevationInputPoints;
        keyframeSubmapTotalRawPointCount += finiteRawPoints;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Keyframe raw submap accumulation: processed_scans=%zu keyframe_submaps=%zu raw_points=%zu localization_voxels=%zu elevation_cells=%zu skipped_empty=%zu assigned_new_keyframe=%d",
            keyframeSubmapProcessedScanCount, keyframeMapSubmaps.size(), keyframeSubmapTotalRawPointCount,
            totalLocalizationVoxelCount(keyframeMapSubmaps), totalElevationCellCount(keyframeMapSubmaps),
            keyframeSubmapSkippedEmptyScanCount, assignedToNewKeyframe ? 1 : 0);
    }

    template <typename PointT>
    bool savePcdFile(const std::filesystem::path &path, const pcl::PointCloud<PointT> &cloud) const
    {
        const int ret = pcl::io::savePCDFileBinary(path.string(), cloud);
        if (ret != 0)
            RCLCPP_ERROR(get_logger(), "Failed to save PCD: %s", path.string().c_str());
        return ret == 0;
    }

    bool interpolatePoseForStamp(double stamp, const pcl::PointCloud<PointTypePose> &keyPoses, PointTypePose &poseOut) const
    {
        if (keyPoses.empty())
            return false;
        if (keyPoses.size() == 1U || stamp <= keyPoses.front().time)
        {
            poseOut = keyPoses.front();
            return true;
        }
        if (stamp >= keyPoses.back().time)
        {
            poseOut = keyPoses.back();
            return true;
        }

        // scan時刻を挟む補正後keyframeを探し、位置は線形、姿勢はquaternion slerpで補間する。
        for (size_t i = 1; i < keyPoses.size(); ++i)
        {
            const auto &after = keyPoses.points[i];
            if (stamp > after.time)
                continue;
            const auto &before = keyPoses.points[i - 1];
            const double duration = std::max(after.time - before.time, 1.0e-9);
            const double ratio = std::clamp((stamp - before.time) / duration, 0.0, 1.0);

            poseOut = before;
            poseOut.x = before.x + static_cast<float>((after.x - before.x) * ratio);
            poseOut.y = before.y + static_cast<float>((after.y - before.y) * ratio);
            poseOut.z = before.z + static_cast<float>((after.z - before.z) * ratio);
            poseOut.time = stamp;

            tf2::Quaternion qBefore;
            tf2::Quaternion qAfter;
            qBefore.setRPY(before.roll, before.pitch, before.yaw);
            qAfter.setRPY(after.roll, after.pitch, after.yaw);
            qBefore.normalize();
            qAfter.normalize();
            tf2::Quaternion qInterp = qBefore.slerp(qAfter, ratio);
            qInterp.normalize();
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            tf2::Matrix3x3(qInterp).getRPY(roll, pitch, yaw);
            poseOut.roll = static_cast<float>(roll);
            poseOut.pitch = static_cast<float>(pitch);
            poseOut.yaw = static_cast<float>(yaw);
            return true;
        }
        poseOut = keyPoses.back();
        return true;
    }

    std::string currentUtcTimestamp() const
    {
        const auto now = std::chrono::system_clock::now();
        const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
        std::tm utcTime{};
        gmtime_r(&nowTime, &utcTime);
        std::ostringstream stream;
        stream << std::put_time(&utcTime, "%FT%TZ");
        return stream.str();
    }

    size_t totalLocalizationVoxelCount(const std::vector<KeyframeMapSubmap> &submaps) const
    {
        size_t count = 0;
        for (const auto &submap : submaps)
            count += submap.localization_voxels.size();
        return count;
    }

    size_t totalElevationCellCount(const std::vector<KeyframeMapSubmap> &submaps) const
    {
        size_t count = 0;
        for (const auto &submap : submaps)
            count += submap.elevation_cells.size();
        return count;
    }

    size_t totalAccumulatedSubmapScanCount(const std::vector<KeyframeMapSubmap> &submaps) const
    {
        size_t count = 0;
        for (const auto &submap : submaps)
            count += submap.scan_count;
        return count;
    }

    bool buildLocalizationCloudFromSubmaps(
        const std::vector<KeyframeMapSubmap> &submaps,
        const pcl::PointCloud<PointTypePose> &keyPoses,
        double globalResolution,
        pcl::PointCloud<PointType> &localizationCloud) const
    {
        localizationCloud.clear();
        if (submaps.size() != keyPoses.size())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Cannot save localization map: keyframe submap count (%zu) does not match pose count (%zu).",
                submaps.size(), keyPoses.size());
            return false;
        }

        std::unordered_map<LocalizationVoxelKey, LocalizationVoxelStats, LocalizationVoxelKeyHash> globalVoxels;
        if (globalResolution > 0.0)
            globalVoxels.reserve(totalLocalizationVoxelCount(submaps));

        // loop closure補正後poseで各keyframe local submapをglobalへ戻し、保存解像度で再voxel化する。
        for (const auto &submap : submaps)
        {
            if (submap.keyframe_index >= keyPoses.size())
            {
                RCLCPP_ERROR(get_logger(), "Cannot save localization map: invalid keyframe submap index %zu.", submap.keyframe_index);
                return false;
            }
            const Eigen::Affine3f mapFromKeyframe = pclPointToAffine3f(keyPoses.points[submap.keyframe_index]);
            for (const auto &entry : submap.localization_voxels)
            {
                const PointType localPoint = entry.second.centroidPoint();
                const Eigen::Vector3f globalPoint = mapFromKeyframe * Eigen::Vector3f(localPoint.x, localPoint.y, localPoint.z);
                if (!std::isfinite(globalPoint.x()) || !std::isfinite(globalPoint.y()) || !std::isfinite(globalPoint.z()))
                    continue;
                if (globalResolution > 0.0)
                {
                    const LocalizationVoxelKey key{
                        static_cast<int64_t>(std::floor(static_cast<double>(globalPoint.x()) / globalResolution)),
                        static_cast<int64_t>(std::floor(static_cast<double>(globalPoint.y()) / globalResolution)),
                        static_cast<int64_t>(std::floor(static_cast<double>(globalPoint.z()) / globalResolution))};
                    globalVoxels[key].addWeighted(globalPoint, entry.second.sum_intensity, entry.second.count);
                }
                else
                {
                    PointType outputPoint = localPoint;
                    outputPoint.x = globalPoint.x();
                    outputPoint.y = globalPoint.y();
                    outputPoint.z = globalPoint.z();
                    localizationCloud.push_back(outputPoint);
                }
            }
        }

        if (globalResolution > 0.0)
        {
            localizationCloud.reserve(globalVoxels.size());
            for (const auto &entry : globalVoxels)
                localizationCloud.push_back(entry.second.centroidPoint());
        }
        return true;
    }

    bool writeElevationManifest(
        const std::filesystem::path &saveDirectory,
        size_t processedScanCount,
        size_t skippedEmptyScanCount,
        const std::vector<KeyframeMapSubmap> &submaps,
        size_t keyframeCount,
        size_t localizationPointCount,
        size_t globalElevationCellCount) const
    {
        const auto manifestPath = saveDirectory / "elevation_manifest.yaml";
        std::ofstream manifest(manifestPath);
        if (!manifest)
        {
            RCLCPP_ERROR(get_logger(), "Failed to open elevation manifest: %s", manifestPath.string().c_str());
            return false;
        }

        // SaveMap responseはsuccessのみのため、成果物の意味と生成時パラメータをmanifestへ集約する。
        manifest << "format_version: 1\n";
        manifest << "created_at: " << currentUtcTimestamp() << "\n";
        manifest << "source: lio_sam/save_map\n";
        manifest << "localization_source: cloudInfo.cloud_deskewed\n";
        manifest << "localization_submap_coordinate: keyframe_local_xyz\n";
        manifest << "elevation_submap_coordinate: gravity_aligned_keyframe_xy_relative_z\n";
        manifest << "correction: optimized_keyframe_pose_rigid_submap\n";
        manifest << "save_lio_sam_standard_pcds: " << (saveLioSamStandardPcds ? "true" : "false") << "\n";
        manifest << "frame_id: " << odometryFrame << "\n";
        manifest << "localization_pcd: localization_map.pcd\n";
        manifest << "global_elevation_csv: global_elevation_map.csv\n";
        manifest << "parameters:\n";
        manifest << "  localization_submap_leaf_size: " << localizationSubmapLeafSize << "\n";
        manifest << "  elevation_output_cell_size: " << elevationOutputCellSize << "\n";
        manifest << "  elevation_cluster_cell_size: " << elevationClusterCellSize << "\n";
        manifest << "  cell_z_cluster_gap: " << elevationCellZClusterGap << "\n";
        manifest << "  cluster_connection_radius: " << elevationClusterConnectionRadius << "\n";
        manifest << "  cluster_connection_z_gap: " << elevationClusterConnectionZGap << "\n";
        manifest << "  elevation_min_range: " << elevationMinRange << "\n";
        manifest << "  elevation_max_range: " << elevationMaxRange << "\n";
        manifest << "counts:\n";
        manifest << "  keyframes: " << keyframeCount << "\n";
        manifest << "  processed_scans: " << processedScanCount << "\n";
        manifest << "  skipped_empty_scans: " << skippedEmptyScanCount << "\n";
        manifest << "  accumulated_scans: " << totalAccumulatedSubmapScanCount(submaps) << "\n";
        manifest << "  keyframe_submaps: " << submaps.size() << "\n";
        manifest << "  localization_submap_voxels: " << totalLocalizationVoxelCount(submaps) << "\n";
        manifest << "  localization_points: " << localizationPointCount << "\n";
        manifest << "  elevation_submap_cells: " << totalElevationCellCount(submaps) << "\n";
        manifest << "  global_elevation_cells: " << globalElevationCellCount << "\n";
        return static_cast<bool>(manifest);
    }

    bool writeGlobalElevationMap(
        const std::filesystem::path &saveDirectory,
        const std::vector<KeyframeMapSubmap> &submaps,
        const pcl::PointCloud<PointTypePose> &keyPoses,
        size_t *globalCellCount) const
    {
        std::unordered_map<ElevationGridKey, ElevationSubmapCell, ElevationGridKeyHash> globalCells;
        globalCells.reserve(totalElevationCellCount(submaps));

        // elevation submapもkeyframe localの剛体mapとして扱い、補正後keyframe poseでglobal cellへ再配置する。
        for (const auto &submap : submaps)
        {
            if (submap.keyframe_index >= keyPoses.size())
            {
                RCLCPP_ERROR(get_logger(), "Cannot save elevation map: invalid keyframe submap index %zu.", submap.keyframe_index);
                return false;
            }
            const auto &pose = keyPoses.points[submap.keyframe_index];
            const double cosYaw = std::cos(pose.yaw);
            const double sinYaw = std::sin(pose.yaw);

            for (const auto &entry : submap.elevation_cells)
            {
                const auto &cell = entry.second;
                const double globalX = pose.x + cosYaw * cell.x - sinYaw * cell.y;
                const double globalY = pose.y + sinYaw * cell.x + cosYaw * cell.y;
                const ElevationGridKey key{
                    static_cast<int64_t>(std::floor(globalX / elevationOutputCellSize)),
                    static_cast<int64_t>(std::floor(globalY / elevationOutputCellSize))};
                ElevationSubmapCell adjustedCell = cell;
                adjustedCell.ix = key.ix;
                adjustedCell.iy = key.iy;
                adjustedCell.x = (static_cast<double>(key.ix) + 0.5) * elevationOutputCellSize;
                adjustedCell.y = (static_cast<double>(key.iy) + 0.5) * elevationOutputCellSize;
                for (auto &cluster : adjustedCell.z_clusters)
                {
                    cluster.z_min += pose.z;
                    cluster.z_max += pose.z;
                    cluster.z_mean += pose.z;
                }

                auto &targetCell = globalCells[key];
                targetCell.ix = key.ix;
                targetCell.iy = key.iy;
                targetCell.x = adjustedCell.x;
                targetCell.y = adjustedCell.y;
                mergeElevationCell(targetCell, adjustedCell);
            }
        }

        std::vector<ElevationGridKey> sortedKeys;
        sortedKeys.reserve(globalCells.size());
        for (const auto &entry : globalCells)
            sortedKeys.push_back(entry.first);
        std::sort(sortedKeys.begin(), sortedKeys.end(), [](const auto &lhs, const auto &rhs) {
            if (lhs.ix != rhs.ix)
                return lhs.ix < rhs.ix;
            return lhs.iy < rhs.iy;
        });

        const auto csvPath = saveDirectory / "global_elevation_map.csv";
        std::ofstream csv(csvPath);
        if (!csv)
        {
            RCLCPP_ERROR(get_logger(), "Failed to open global elevation CSV: %s", csvPath.string().c_str());
            return false;
        }

        std::unordered_map<ElevationGridKey, std::vector<ElevationRunningStats>, ElevationGridKeyHash> globalClusterCells;
        globalClusterCells.reserve(globalCells.size());
        for (const auto &entry : globalCells)
        {
            const auto &cell = entry.second;
            const ElevationGridKey clusterKey{
                static_cast<int64_t>(std::floor(cell.x / elevationClusterCellSize)),
                static_cast<int64_t>(std::floor(cell.y / elevationClusterCellSize))};
            mergeElevationClusters(globalClusterCells[clusterKey], cell.z_clusters);
        }
        const auto selectedClusterCells = selectOriginConnectedElevationClusters(globalClusterCells);

        csv << "ix,iy,x,y,count,z_min,z_max,z_mean,z_m2,lowest_cluster_count,lowest_cluster_min,lowest_cluster_max,lowest_cluster_mean,height_range\n";
        csv << std::fixed << std::setprecision(6);
        size_t writtenCellCount = 0;
        for (const auto &key : sortedKeys)
        {
            const auto &cell = globalCells.at(key);
            const ElevationGridKey clusterKey{
                static_cast<int64_t>(std::floor(cell.x / elevationClusterCellSize)),
                static_cast<int64_t>(std::floor(cell.y / elevationClusterCellSize))};
            const auto clusterIter = globalClusterCells.find(clusterKey);
            if (clusterIter == globalClusterCells.end() || clusterIter->second.empty())
                continue;
            const auto selectedIter = selectedClusterCells.find(clusterKey);
            if (selectedIter == selectedClusterCells.end() || selectedIter->second >= clusterIter->second.size())
                continue;
            const auto &selectedGlobalCluster = clusterIter->second[selectedIter->second];

            ElevationRunningStats selectedStats;
            for (const auto &cluster : cell.z_clusters)
            {
                if (elevationClusterOverlaps(cluster, selectedGlobalCluster))
                    selectedStats.merge(cluster.count, cluster.z_min, cluster.z_max, cluster.z_mean, cluster.z_m2);
            }
            if (selectedStats.count == 0)
                continue;
            csv << key.ix << ',' << key.iy << ','
                << (static_cast<double>(key.ix) + 0.5) * elevationOutputCellSize << ','
                << (static_cast<double>(key.iy) + 0.5) * elevationOutputCellSize << ','
                << selectedStats.count << ',' << selectedStats.z_min << ',' << selectedStats.z_max << ',' << selectedStats.z_mean << ',' << selectedStats.z_m2 << ','
                << selectedStats.count << ',' << selectedStats.z_min << ',' << selectedStats.z_max << ',' << selectedStats.z_mean << ','
                << selectedStats.z_max - selectedStats.z_min << '\n';
            ++writtenCellCount;
        }
        if (!csv)
        {
            RCLCPP_ERROR(get_logger(), "Failed to write global elevation CSV: %s", csvPath.string().c_str());
            return false;
        }

        if (globalCellCount != nullptr)
            *globalCellCount = writtenCellCount;
        RCLCPP_INFO(
            get_logger(),
            "Elevation map save stats: keyframe_submaps=%zu elevation_submap_cells=%zu global_output_candidate_cells=%zu global_cluster_cells=%zu selected_cluster_cells=%zu global_elevation_cells=%zu",
            submaps.size(), totalElevationCellCount(submaps), sortedKeys.size(), globalClusterCells.size(), selectedClusterCells.size(), writtenCellCount);
        return true;
    }

    bool saveMapToFiles(const std::shared_ptr<lio_sam::srv::SaveMap::Request> req)
    {
        const auto saveDirectory = resolveSaveMapDirectory(req->destination);
        pcl::PointCloud<PointType>::Ptr keyPoses3D(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointTypePose>::Ptr keyPoses6D(new pcl::PointCloud<PointTypePose>());
        std::vector<pcl::PointCloud<PointType>::Ptr> cornerFrames;
        std::vector<pcl::PointCloud<PointType>::Ptr> surfFrames;
        std::vector<KeyframeMapSubmap> submaps;
        size_t processedScanCount = 0;
        size_t skippedEmptyScanCount = 0;

        {
            std::lock_guard<std::mutex> lock(mtx);
            if (cloudKeyPoses3D->empty() || cloudKeyPoses6D->empty())
            {
                RCLCPP_ERROR(get_logger(), "Cannot save map: no LIO-SAM keyframes are available.");
                return false;
            }

            // 保存処理中にloop closure threadがkeyframeを更新しても一貫したsnapshotで出力する。
            *keyPoses3D = *cloudKeyPoses3D;
            *keyPoses6D = *cloudKeyPoses6D;
            if (saveLioSamStandardPcds)
            {
                cornerFrames.reserve(cornerCloudKeyFrames.size());
                surfFrames.reserve(surfCloudKeyFrames.size());
                for (const auto &cloud : cornerCloudKeyFrames)
                    cornerFrames.push_back(pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>(*cloud)));
                for (const auto &cloud : surfCloudKeyFrames)
                    surfFrames.push_back(pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>(*cloud)));
            }
            submaps = keyframeMapSubmaps;
            processedScanCount = keyframeSubmapProcessedScanCount;
            skippedEmptyScanCount = keyframeSubmapSkippedEmptyScanCount;
        }

        if (submaps.size() != keyPoses6D->size())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Cannot save map: keyframe submap count (%zu) does not match pose count (%zu). Is saveElevationMap enabled from startup?",
                submaps.size(), keyPoses6D->size());
            return false;
        }
        if (saveLioSamStandardPcds && (cornerFrames.size() != keyPoses6D->size() || surfFrames.size() != keyPoses6D->size()))
        {
            RCLCPP_ERROR(get_logger(), "Cannot save map: keyframe cloud count does not match pose count.");
            return false;
        }

        try
        {
            std::filesystem::remove_all(saveDirectory);
            std::filesystem::create_directories(saveDirectory);
        }
        catch (const std::filesystem::filesystem_error &error)
        {
            RCLCPP_ERROR(get_logger(), "Failed to prepare save directory %s: %s", saveDirectory.string().c_str(), error.what());
            return false;
        }

        RCLCPP_INFO(get_logger(), "Saving map to %s", saveDirectory.string().c_str());
        bool ok = true;

        if (saveLioSamStandardPcds)
        {
            ok = savePcdFile(saveDirectory / "trajectory.pcd", *keyPoses3D) && ok;
            ok = savePcdFile(saveDirectory / "transformations.pcd", *keyPoses6D) && ok;

            pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
            for (size_t i = 0; i < keyPoses6D->size(); ++i)
            {
                *globalCornerCloud += *transformPointCloud(cornerFrames[i], &keyPoses6D->points[i]);
                *globalSurfCloud += *transformPointCloud(surfFrames[i], &keyPoses6D->points[i]);
            }
            *globalMapCloud += *globalCornerCloud;
            *globalMapCloud += *globalSurfCloud;

            if (req->resolution > 0.0)
            {
                pcl::VoxelGrid<PointType> cornerFilter;
                pcl::VoxelGrid<PointType> surfFilter;
                pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
                const float leafSize = static_cast<float>(req->resolution);

                // 標準PCD群を明示保存する場合だけ、従来どおりfeature mapを生成してdownsampleする。
                cornerFilter.setLeafSize(leafSize, leafSize, leafSize);
                cornerFilter.setInputCloud(globalCornerCloud);
                cornerFilter.filter(*globalCornerCloudDS);
                surfFilter.setLeafSize(leafSize, leafSize, leafSize);
                surfFilter.setInputCloud(globalSurfCloud);
                surfFilter.filter(*globalSurfCloudDS);
                ok = savePcdFile(saveDirectory / "CornerMap.pcd", *globalCornerCloudDS) && ok;
                ok = savePcdFile(saveDirectory / "SurfMap.pcd", *globalSurfCloudDS) && ok;
            }
            else
            {
                ok = savePcdFile(saveDirectory / "CornerMap.pcd", *globalCornerCloud) && ok;
                ok = savePcdFile(saveDirectory / "SurfMap.pcd", *globalSurfCloud) && ok;
            }

            ok = savePcdFile(saveDirectory / "GlobalMap.pcd", *globalMapCloud) && ok;
        }

        pcl::PointCloud<PointType> localizationCloud;
        if (!buildLocalizationCloudFromSubmaps(submaps, *keyPoses6D, req->resolution, localizationCloud))
            return false;
        if (localizationCloud.empty())
        {
            RCLCPP_ERROR(get_logger(), "Cannot save map: localization raw submap is empty.");
            return false;
        }
        ok = savePcdFile(saveDirectory / "localization_map.pcd", localizationCloud) && ok;

        size_t globalElevationCellCount = 0;
        ok = writeGlobalElevationMap(saveDirectory, submaps, *keyPoses6D, &globalElevationCellCount) && ok;
        ok = writeElevationManifest(
            saveDirectory,
            processedScanCount,
            skippedEmptyScanCount,
            submaps,
            keyPoses6D->size(),
            localizationCloud.size(),
            globalElevationCellCount) && ok;

        RCLCPP_INFO(
            get_logger(),
            "Map save completed: success=%d keyframes=%zu keyframe_submaps=%zu localization_points=%zu elevation_cells=%zu output=%s",
            ok ? 1 : 0, keyPoses6D->size(), submaps.size(), localizationCloud.size(), globalElevationCellCount,
            saveDirectory.string().c_str());
        return ok;
    }

    void laserCloudInfoHandler(const lio_sam::msg::CloudInfo::SharedPtr msgIn)
    {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = stamp2Sec(msgIn->header.stamp);

        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;

            updateInitialGuess();

            extractSurroundingKeyFrames();

            downsampleCurrentScan();

            scan2MapOptimization();

            const size_t keyframeCountBeforeSave = cloudKeyPoses6D->size();
            const PointTypePose scanPoseAfterOptimization = trans2PointTypePose(transformTobeMapped);

            saveKeyFramesAndFactor();

            accumulateKeyframeRawSubmap(keyframeCountBeforeSave, cloudKeyPoses6D->size(), scanPoseAfterOptimization);

            correctPoses();

            publishOdometry();

            publishFrames();
        }
        else
        {
            // mappingProcessIntervalでscanを処理しない場合も、地図密度低下の原因として追えるようにする。
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Skip CloudInfo in mapOptimization due to mappingProcessInterval: stamp=%.6f last=%.6f interval=%.6f delta=%.6f",
                timeLaserInfoCur, timeLastProcessing, mappingProcessInterval, timeLaserInfoCur - timeLastProcessing);
        }
    }

    void gpsHandler(const nav_msgs::msg::Odometry::SharedPtr gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,  Eigen::Affine3f transCur)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint) const
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[]) const
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[]) const
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    void visualizeGlobalMapThread()
    {
        rclcpp::Rate rate(0.2);
        while (rclcpp::ok()){
            rate.sleep();
            publishGlobalMap();
        }
        if (savePCD == false)
            return;
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;
        savePCDDirectory = std::getenv("HOME") + savePCDDirectory;
        int unused = system((std::string("exec rm -r ") + savePCDDirectory).c_str());
        unused = system((std::string("mkdir ") + savePCDDirectory).c_str());
        pcl::io::savePCDFileASCII(savePCDDirectory + "trajectory.pcd", *cloudKeyPoses3D);
        pcl::io::savePCDFileASCII(savePCDDirectory + "transformations.pcd", *cloudKeyPoses6D);
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
            *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
            *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
            cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
        }
        downSizeFilterCorner.setInputCloud(globalCornerCloud);
        downSizeFilterCorner.filter(*globalCornerCloudDS);
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudCorner.pcd", *globalCornerCloudDS);
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudSurf.pcd", *globalSurfCloudDS);
        *globalMapCloud += *globalCornerCloud;
        *globalMapCloud += *globalSurfCloud;
        pcl::io::savePCDFileASCII(savePCDDirectory + "cloudGlobal.pcd", *globalMapCloud);
        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed" << endl;
    }

    void publishGlobalMap()
    {
        if (!publishMapGlobalCloud)
            return;

        if (pubLaserCloudSurround == nullptr || pubLaserCloudSurround->get_subscription_count() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }












    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        rclcpp::Rate rate(loopClosureFrequency);
        while (rclcpp::ok())
        {
            rate.sleep();
            performLoopClosure();
            visualizeLoopClosure();
        }
    }

    void loopInfoHandler(const std_msgs::msg::Float64MultiArray::SharedPtr loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2)
            return;

        loopInfoVec.push_back(*loopMsg);

        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    void performLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (publishLoopClosureClouds && pubHistoryKeyFrames != nullptr && pubHistoryKeyFrames->get_subscription_count() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if (publishLoopClosureClouds && pubIcpKeyFrames != nullptr && pubIcpKeyFrames->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty())
            return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();

        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure()
    {
        if (!publishLoopClosureClouds || pubLoopConstraintEdge == nullptr || pubLoopConstraintEdge->get_subscription_count() == 0)
            return;

        if (loopIndexContainer.empty())
            return;

        visualization_msgs::msg::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::msg::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::msg::Marker::ADD;
        markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3;
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::msg::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::msg::Marker::ADD;
        markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::msg::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge->publish(markerArray);
    }











    // AI_SHIP_ROBOT_BEGIN: IMU preintegration初期値を並進・回転で個別に採用する。
    void updateInitialGuess()
    {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        // initialization
        if (cloudKeyPoses3D->points.empty())
        {
            transformTobeMapped[0] = cloudInfo.imu_roll_init;
            transformTobeMapped[1] = cloudInfo.imu_pitch_init;
            transformTobeMapped[2] = cloudInfo.imu_yaw_init;

            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);
            return;
        }

        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odom_available == true && useImuPreintegrationInitialGuess)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(
                cloudInfo.initial_guess_x, cloudInfo.initial_guess_y, cloudInfo.initial_guess_z,
                cloudInfo.initial_guess_roll, cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);
            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            }
            else
            {
                Eigen::Affine3f transIncreRaw = lastImuPreTransformation.inverse() * transBack;
                float xIncre, yIncre, zIncre, rollIncre, pitchIncre, yawIncre;
                pcl::getTranslationAndEulerAngles(transIncreRaw, xIncre, yIncre, zIncre, rollIncre, pitchIncre, yawIncre);

                // 6軸IMUではpreintegration並進が大きく壊れやすいため、並進・回転を独立に初期値へ採用する。
                Eigen::Affine3f transIncre = pcl::getTransformation(
                    useImuTranslationInitialGuess ? xIncre : 0.0f,
                    useImuTranslationInitialGuess ? yIncre : 0.0f,
                    useImuTranslationInitialGuess ? zIncre : 0.0f,
                    useImuRotationInitialGuess ? rollIncre : 0.0f,
                    useImuRotationInitialGuess ? pitchIncre : 0.0f,
                    useImuRotationInitialGuess ? yawIncre : 0.0f);
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "IMU initial guess delta: trans=(%.4f, %.4f, %.4f) norm=%.4f rot=(%.4f, %.4f, %.4f) use_trans=%d use_rot=%d",
                    xIncre, yIncre, zIncre, std::sqrt(xIncre*xIncre + yIncre*yIncre + zIncre*zIncre),
                    rollIncre, pitchIncre, yawIncre, useImuTranslationInitialGuess ? 1 : 0, useImuRotationInitialGuess ? 1 : 0);

                lastImuPreTransformation = transBack;
                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);
                return;
            }
        }
        else if (cloudInfo.odom_available == true)
        {
            lastImuPreTransformation = pcl::getTransformation(
                cloudInfo.initial_guess_x, cloudInfo.initial_guess_y, cloudInfo.initial_guess_z,
                cloudInfo.initial_guess_roll, cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);
            lastImuPreTransAvailable = true;
        }

        // preintegration初期値を使わない場合でも、IMU角速度由来の相対回転だけはfallbackとして使えるようにする。
        if (cloudInfo.imu_available == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);
            return;
        }
    }
    // AI_SHIP_ROBOT_END

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for(auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end())
            {
                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }

        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    void downsampleCurrentScan()
    {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
            // lidar -> camera
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 20; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;
            }

            transformUpdate();
        } else {
            RCLCPP_WARN(get_logger(), "Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    // AI_SHIP_ROBOT_BEGIN: transformUpdateとincremental odometryで同じroll/pitch融合を使う。
    void blendRollPitchWithImu(float *roll, float *pitch)
    {
        if (cloudInfo.imu_available != true || std::abs(cloudInfo.imu_pitch_init) >= 1.4)
            return;

        // scan matching結果とincremental odometryで同じ重みを使い、roll/pitch補正の出力差を避ける。
        const double imuWeight = imuRPYWeight;
        tf2::Quaternion imuQuaternion;
        tf2::Quaternion transformQuaternion;
        double rollMid, pitchMid, yawMid;

        transformQuaternion.setRPY(*roll, 0, 0);
        imuQuaternion.setRPY(cloudInfo.imu_roll_init, 0, 0);
        tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
        *roll = rollMid;

        transformQuaternion.setRPY(0, *pitch, 0);
        imuQuaternion.setRPY(0, cloudInfo.imu_pitch_init, 0);
        tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
        *pitch = pitchMid;
    }

    void transformUpdate()
    {
        blendRollPitchWithImu(&transformTobeMapped[0], &transformTobeMapped[1]);

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }
    // AI_SHIP_ROBOT_END

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        if (sensor == SensorType::LIVOX)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0)
                return true;
        }

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addGPSFactor()
    {
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        // pose covariance small, no need to correct
        if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (stamp2Sec(gpsQueue.front().header.stamp) < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (stamp2Sec(gpsQueue.front().header.stamp) > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::msg::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;
                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
            return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = rclcpp::Time(pose_in.time * 1e9);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf2::Quaternion q;
        q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    double elapsedMilliseconds(
        const std::chrono::steady_clock::time_point& start,
        const std::chrono::steady_clock::time_point& end) const
    {
        return std::chrono::duration<double, std::milli>(end - start).count();
    }

    void resetHybridCloudTimingAccumulator()
    {
        hybridCloudTimingCount = 0;
        hybridCloudFromRosMsSum = 0.0;
        hybridCloudAppendMsSum = 0.0;
        hybridCloudPublishMsSum = 0.0;
        hybridCloudTotalMsSum = 0.0;
        hybridCloudRawNearPointsSum = 0;
        hybridCloudFeaturePointsSum = 0;
        hybridCloudOutputPointsSum = 0;
    }

    void recordHybridCloudTiming(
        const double fromRosMs,
        const double appendMs,
        const double publishMs,
        const double totalMs,
        const size_t rawNearPoints,
        const size_t featurePoints,
        const size_t outputPoints)
    {
        if (processingTimeLogIntervalSec <= 0.0)
            return;

        // hybrid生成はmap保存品質とCPU負荷の両方に効くため、平均時間と点数をまとめて記録する。
        ++hybridCloudTimingCount;
        hybridCloudFromRosMsSum += fromRosMs;
        hybridCloudAppendMsSum += appendMs;
        hybridCloudPublishMsSum += publishMs;
        hybridCloudTotalMsSum += totalMs;
        hybridCloudRawNearPointsSum += rawNearPoints;
        hybridCloudFeaturePointsSum += featurePoints;
        hybridCloudOutputPointsSum += outputPoints;

        const auto now = std::chrono::steady_clock::now();
        const double elapsedSec = std::chrono::duration<double>(now - lastHybridCloudTimingLogTime).count();
        if (elapsedSec < processingTimeLogIntervalSec)
            return;

        const double count = static_cast<double>(std::max<uint64_t>(hybridCloudTimingCount, 1));
        RCLCPP_INFO(
            get_logger(),
            "Hybrid cloud timing: scans=%lu avg_ms total=%.3f from_ros=%.3f append=%.3f publish=%.3f points raw_near=%.1f feature=%.1f output=%.1f",
            static_cast<unsigned long>(hybridCloudTimingCount),
            hybridCloudTotalMsSum / count,
            hybridCloudFromRosMsSum / count,
            hybridCloudAppendMsSum / count,
            hybridCloudPublishMsSum / count,
            static_cast<double>(hybridCloudRawNearPointsSum) / count,
            static_cast<double>(hybridCloudFeaturePointsSum) / count,
            static_cast<double>(hybridCloudOutputPointsSum) / count);
        lastHybridCloudTimingLogTime = now;
        resetHybridCloudTimingAccumulator();
    }

    Eigen::Affine3f transformStampedToAffine3f(const geometry_msgs::msg::TransformStamped& stampedTransform) const
    {
        // TF messageを点ごとの高さ判定で使いやすいfloat affineへ変換する。
        const auto& translation = stampedTransform.transform.translation;
        const auto& rotation = stampedTransform.transform.rotation;
        tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
        quaternion.normalize();
        tf2::Matrix3x3 matrix(quaternion);

        Eigen::Affine3f affine = Eigen::Affine3f::Identity();
        affine(0, 0) = static_cast<float>(matrix[0][0]);
        affine(0, 1) = static_cast<float>(matrix[0][1]);
        affine(0, 2) = static_cast<float>(matrix[0][2]);
        affine(1, 0) = static_cast<float>(matrix[1][0]);
        affine(1, 1) = static_cast<float>(matrix[1][1]);
        affine(1, 2) = static_cast<float>(matrix[1][2]);
        affine(2, 0) = static_cast<float>(matrix[2][0]);
        affine(2, 1) = static_cast<float>(matrix[2][1]);
        affine(2, 2) = static_cast<float>(matrix[2][2]);
        affine(0, 3) = static_cast<float>(translation.x);
        affine(1, 3) = static_cast<float>(translation.y);
        affine(2, 3) = static_cast<float>(translation.z);
        return affine;
    }

    bool lookupMapFromOdometryTransform(Eigen::Affine3f* mapFromOdometry)
    {
        if (mapFromOdometry == nullptr)
            return false;
        if (mapFrame == odometryFrame)
        {
            *mapFromOdometry = Eigen::Affine3f::Identity();
            return true;
        }

        try
        {
            // mapFrame上限はTF未接続時に旧挙動を壊さないため、取得失敗時は呼び出し側でfail-openにする。
            const auto stampedTransform = tfBuffer.lookupTransform(
                mapFrame, odometryFrame, tf2_ros::fromRclcpp(timeLaserInfoStamp), tf2::durationFromSec(tfTimeoutSec));
            *mapFromOdometry = transformStampedToAffine3f(stampedTransform);
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Failed to lookup transform %s <- %s for hybrid raw near map Z limit; publish without height filter: %s",
                mapFrame.c_str(), odometryFrame.c_str(), ex.what());
            return false;
        }
    }

    bool buildHybridRawNearLocalToMapTransform(Eigen::Affine3f* rawNearLocalToMap)
    {
        if (rawNearLocalToMap == nullptr)
            return false;

        Eigen::Affine3f mapFromOdometry;
        if (!lookupMapFromOdometryTransform(&mapFromOdometry))
            return false;

        // raw近傍点群はlidarFrame localなので、現在推定pose経由でmapFrame高さへ写す。
        *rawNearLocalToMap = mapFromOdometry * trans2Affine3f(transformTobeMapped);
        return true;
    }

    size_t appendHeightFilteredRawNearPoints(
        pcl::PointCloud<PointType>::Ptr cloudOut,
        const pcl::PointCloud<PointType>::Ptr cloudIn,
        const Eigen::Affine3f* rawNearLocalToMap)
    {
        size_t appendedCount = 0;
        // 高さ制限が有効な時だけmapFrame Zを評価し、publishする点の座標系自体はlocal frameのまま残す。
        for (const auto &point : cloudIn->points)
        {
            if (rawNearLocalToMap != nullptr)
            {
                const Eigen::Vector3f mapPoint = (*rawNearLocalToMap) * Eigen::Vector3f(point.x, point.y, point.z);
                if (mapPoint.z() > hybridRegisteredCloudRawUpperMapZMax)
                    continue;
            }

            cloudOut->push_back(point);
            ++appendedCount;
        }
        return appendedCount;
    }

    size_t appendFeaturePoints(
        pcl::PointCloud<PointType>::Ptr cloudOut,
        const pcl::PointCloud<PointType>::Ptr cloudIn)
    {
        size_t appendedCount = 0;
        // 上側高さ制限はraw近傍詳細点だけに適用し、SLAM用の粗いfeature点群は全体形状として保持する。
        for (const auto &point : cloudIn->points)
        {
            cloudOut->push_back(point);
            ++appendedCount;
        }
        return appendedCount;
    }

    // AI_SHIP_ROBOT_BEGIN: 公開odometry/TFをodometryFrame->lidarFrameに統一し、incrementalにも同じIMU融合を使う。
    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::msg::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = lidarFrame;
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        laserOdometryROS.pose.pose.orientation = quat_msg;
        pubLaserOdometryGlobal->publish(laserOdometryROS);

        // Publish TF
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        tf2::Transform t_odom_to_lidar = tf2::Transform(quat_tf, tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf2::TimePoint time_point = tf2_ros::fromRclcpp(timeLaserInfoStamp);
        tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point, odometryFrame);
        geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
        tf2::convert(temp_odom_to_lidar, trans_odom_to_lidar);
        trans_odom_to_lidar.child_frame_id = lidarFrame;
        br->sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::msg::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            blendRollPitchWithImu(&roll, &pitch);
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = lidarFrame;
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);
            laserOdomIncremental.pose.pose.orientation = quat_msg;
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental->publish(laserOdomIncremental);
    }
    // AI_SHIP_ROBOT_END

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        if (publishTrajectoryCloud && pubKeyPoses != nullptr && pubKeyPoses->get_subscription_count() != 0)
            publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        if (publishMapLocalCloud && pubRecentKeyFrames != nullptr && pubRecentKeyFrames->get_subscription_count() != 0)
            publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (publishCloudRegistered && pubRecentKeyFrame != nullptr && pubRecentKeyFrame->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            // 蓄積ノード側でpose変換できるよう、登録済みfeature点群はscan local frameのままpublishする。
            *cloudOut += *laserCloudCornerLastDS;
            *cloudOut += *laserCloudSurfLastDS;
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, lidarFrame);
        }
        // publish registered high-res raw cloud
        if (publishCloudRegisteredRaw && pubCloudRegisteredRaw != nullptr && pubCloudRegisteredRaw->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, lidarFrame);
        }
        // PCD map用hybrid点群は高さ制限後の近傍raw詳細点群とSLAM用feature点群全体をscan local frameで合成する。
        if (hybridRegisteredCloudEnabled && pubRecentKeyFrameHybrid != nullptr && pubRecentKeyFrameHybrid->get_subscription_count() != 0)
        {
            const auto totalStartTime = std::chrono::steady_clock::now();
            pcl::PointCloud<PointType>::Ptr rawNearCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

            const auto fromRosStartTime = std::chrono::steady_clock::now();
            if (cloudInfo.cloud_deskewed_raw_near.width * cloudInfo.cloud_deskewed_raw_near.height > 0U)
                pcl::fromROSMsg(cloudInfo.cloud_deskewed_raw_near, *rawNearCloud);
            const auto fromRosEndTime = std::chrono::steady_clock::now();

            const auto appendStartTime = std::chrono::steady_clock::now();
            Eigen::Affine3f rawNearLocalToMap;
            const Eigen::Affine3f* rawNearLocalToMapPtr = nullptr;
            if (hybridRegisteredCloudRawUpperMapZLimitEnabled && !rawNearCloud->empty() &&
                buildHybridRawNearLocalToMapTransform(&rawNearLocalToMap))
            {
                rawNearLocalToMapPtr = &rawNearLocalToMap;
            }
            // 高さ制限はraw近傍だけに掛け、粗いfeature点群は近傍/遠方を問わず残す。
            const size_t rawNearPoints = appendHeightFilteredRawNearPoints(cloudOut, rawNearCloud, rawNearLocalToMapPtr);
            const size_t cornerFeaturePoints = appendFeaturePoints(cloudOut, laserCloudCornerLastDS);
            const size_t surfFeaturePoints = appendFeaturePoints(cloudOut, laserCloudSurfLastDS);
            const size_t featurePoints = cornerFeaturePoints + surfFeaturePoints;
            const auto appendEndTime = std::chrono::steady_clock::now();

            const auto publishStartTime = std::chrono::steady_clock::now();
            publishCloud(pubRecentKeyFrameHybrid, cloudOut, timeLaserInfoStamp, lidarFrame);
            const auto publishEndTime = std::chrono::steady_clock::now();

            recordHybridCloudTiming(
                elapsedMilliseconds(fromRosStartTime, fromRosEndTime),
                elapsedMilliseconds(appendStartTime, appendEndTime),
                elapsedMilliseconds(publishStartTime, publishEndTime),
                elapsedMilliseconds(totalStartTime, publishEndTime),
                rawNearPoints,
                featurePoints,
                cloudOut->size());
        }
        // publish path
        if (pubPath->get_subscription_count() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath->publish(globalPath);
        }
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    // TransformListenerが購読する/tf_staticはtransient local QoSなので、intra-processとは併用しない。
    options.use_intra_process_comms(false);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto MO = std::make_shared<mapOptimization>(options);
    exec.add_node(MO);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Map Optimization Started.\033[0m");

    std::thread loopthread(&mapOptimization::loopClosureThread, MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, MO);

    exec.spin();

    rclcpp::shutdown();

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}
