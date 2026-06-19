#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/kdtree/kdtree_flann.h>  // pcl include kdtree_flann throws error if PCL_NO_PRECOMPILE
                                      // is defined before
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <cstdint>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER, LIVOX };

class ParamServer : public rclcpp::Node
{
public:
    std::string robot_id;

    //Topics
    string pointCloudTopic;
    // AI_SHIP_ROBOT_BEGIN: imageProjectionが複数Livox CustomMsgを直接同期するための入力設定。
    vector<string> inputCustomTopics;
    vector<int64_t> inputRingOffsets;
    string referenceCustomTopic;
    double maxStampDeltaSec;
    double tfTimeoutSec;
    double timestampUnitScale;
    // AI_SHIP_ROBOT_END
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor = SensorType::LIVOX;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    // AI_SHIP_ROBOT_BEGIN: 6軸IMU初期化、単位変換、deskew方式切替をparameter化する。
    string imuType;
    string imuAccelerationUnit;
    double imuAccelerationScale;
    double imuFrequency;
    bool imuDebug;
    bool waitForImuInitialization;
    double initialImuExpectedAccelerationNorm;
    double initialImuAccelerationNormTolerance;
    double initialImuMaxAngularVelocity;
    int initialImuMinSamples;
    double initialImuMinDuration;
    bool useImuPreintegrationInitialGuess;
    bool useImuTranslationInitialGuess;
    bool useImuRotationInitialGuess;
    string deskewMode;
    double maxPointOffsetTimeSec;
    bool sixAxisInitialOrientationReady = false;
    double sixAxisInitialRoll = 0.0;
    double sixAxisInitialPitch = 0.0;
    Eigen::Vector3d sixAxisInitialGyroBias = Eigen::Vector3d::Zero();
    Eigen::Vector3d sixAxisAccSum = Eigen::Vector3d::Zero();
    Eigen::Vector3d sixAxisGyroSum = Eigen::Vector3d::Zero();
    double sixAxisAccNormSum = 0.0;
    double sixAxisAccNormSquareSum = 0.0;
    double sixAxisFirstSampleTime = 0.0;
    int sixAxisSampleCount = 0;
    // AI_SHIP_ROBOT_END
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;
    // AI_SHIP_ROBOT_BEGIN: PCD map保存用hybrid点群をlocal frameで出すためのparameterを追加する。
    bool hybridRegisteredCloudEnabled;
    float hybridRegisteredCloudRawNearRange;
    float hybridRegisteredCloudRawNearLeafSize;
    string hybridRegisteredCloudTopic;
    bool publishDeskewedCloud;
    bool publishFeatureClouds;
    bool publishMapGlobalCloud;
    bool publishMapLocalCloud;
    bool publishTrajectoryCloud;
    bool publishCloudRegistered;
    bool publishCloudRegisteredRaw;
    bool publishLoopClosureClouds;
    // AI_SHIP_ROBOT_END

    float z_tollerance;
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;
    // AI_SHIP_ROBOT_BEGIN: rosbag再生backlog時の処理追跡とtimer内batch処理を制御する。
    int imageProjectionMaxScansPerTimer;
    int imageProjectionBacklogLogThreshold;
    double processingTimeLogIntervalSec;
    // AI_SHIP_ROBOT_END

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer(std::string node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
    {
        declare_parameter("pointCloudTopic", "points");
        get_parameter("pointCloudTopic", pointCloudTopic);
        // AI_SHIP_ROBOT_BEGIN: 旧fusion topicではなく、各LiDARのCustomMsg topicをimageProjectionへ渡す。
        declare_parameter("input_custom_topics", std::vector<std::string>{});
        get_parameter("input_custom_topics", inputCustomTopics);
        declare_parameter("input_ring_offsets", std::vector<int64_t>{});
        get_parameter("input_ring_offsets", inputRingOffsets);
        declare_parameter("reference_custom_topic", "");
        get_parameter("reference_custom_topic", referenceCustomTopic);
        declare_parameter("max_stamp_delta_sec", 0.07);
        get_parameter("max_stamp_delta_sec", maxStampDeltaSec);
        declare_parameter("tf_timeout_sec", 0.1);
        get_parameter("tf_timeout_sec", tfTimeoutSec);
        declare_parameter("timestamp_unit_scale", 1.0e-9);
        get_parameter("timestamp_unit_scale", timestampUnitScale);
        // AI_SHIP_ROBOT_END
        declare_parameter("imuTopic", "imu/data");
        get_parameter("imuTopic", imuTopic);
        declare_parameter("odomTopic", "lio_sam/odometry/imu");
        get_parameter("odomTopic", odomTopic);
        declare_parameter("gpsTopic", "lio_sam/odometry/gps");
        get_parameter("gpsTopic", gpsTopic);

        declare_parameter("lidarFrame", "laser_data_frame");
        get_parameter("lidarFrame", lidarFrame);
        declare_parameter("baselinkFrame", "base_link");
        get_parameter("baselinkFrame", baselinkFrame);
        declare_parameter("odometryFrame", "odom");
        get_parameter("odometryFrame", odometryFrame);
        declare_parameter("mapFrame", "map");
        get_parameter("mapFrame", mapFrame);

        declare_parameter("useImuHeadingInitialization", false);
        get_parameter("useImuHeadingInitialization", useImuHeadingInitialization);
        declare_parameter("useGpsElevation", false);
        get_parameter("useGpsElevation", useGpsElevation);
        declare_parameter("gpsCovThreshold", 2.0);
        get_parameter("gpsCovThreshold", gpsCovThreshold);
        declare_parameter("poseCovThreshold", 25.0);
        get_parameter("poseCovThreshold", poseCovThreshold);

        declare_parameter("savePCD", false);
        get_parameter("savePCD", savePCD);
        declare_parameter("savePCDDirectory", "/Downloads/LOAM/");
        get_parameter("savePCDDirectory", savePCDDirectory);

        std::string sensorStr;
        declare_parameter("sensor", "ouster");
        get_parameter("sensor", sensorStr);
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        }
        else
        {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
            rclcpp::shutdown();
        }

        declare_parameter("N_SCAN", 64);
        get_parameter("N_SCAN", N_SCAN);
        declare_parameter("Horizon_SCAN", 512);
        get_parameter("Horizon_SCAN", Horizon_SCAN);
        declare_parameter("downsampleRate", 1);
        get_parameter("downsampleRate", downsampleRate);
        declare_parameter("lidarMinRange", 5.5);
        get_parameter("lidarMinRange", lidarMinRange);
        declare_parameter("lidarMaxRange", 1000.0);
        get_parameter("lidarMaxRange", lidarMaxRange);

        declare_parameter("imuAccNoise", 9e-4);
        get_parameter("imuAccNoise", imuAccNoise);
        declare_parameter("imuGyrNoise", 1.6e-4);
        get_parameter("imuGyrNoise", imuGyrNoise);
        declare_parameter("imuAccBiasN", 5e-4);
        get_parameter("imuAccBiasN", imuAccBiasN);
        declare_parameter("imuGyrBiasN", 7e-5);
        get_parameter("imuGyrBiasN", imuGyrBiasN);
        declare_parameter("imuGravity", 9.80511);
        get_parameter("imuGravity", imuGravity);
        declare_parameter("imuRPYWeight", 0.01);
        get_parameter("imuRPYWeight", imuRPYWeight);
        // AI_SHIP_ROBOT_BEGIN: 実機Livoxのg入力とGazeboのm/s^2入力を同じpreintegrationへ渡す。
        declare_parameter("imuType", "six_axis");
        get_parameter("imuType", imuType);
        declare_parameter("imuAccelerationUnit", "g");
        get_parameter("imuAccelerationUnit", imuAccelerationUnit);
        declare_parameter("imuAccelerationScale", 1.0);
        get_parameter("imuAccelerationScale", imuAccelerationScale);
        declare_parameter("imuFrequency", 500.0);
        get_parameter("imuFrequency", imuFrequency);
        declare_parameter("imuDebug", false);
        get_parameter("imuDebug", imuDebug);
        declare_parameter("waitForImuInitialization", true);
        get_parameter("waitForImuInitialization", waitForImuInitialization);
        declare_parameter("initialImuExpectedAccelerationNorm", 1.0);
        get_parameter("initialImuExpectedAccelerationNorm", initialImuExpectedAccelerationNorm);
        declare_parameter("initialImuAccelerationNormTolerance", 0.35);
        get_parameter("initialImuAccelerationNormTolerance", initialImuAccelerationNormTolerance);
        declare_parameter("initialImuMaxAngularVelocity", 0.2);
        get_parameter("initialImuMaxAngularVelocity", initialImuMaxAngularVelocity);
        declare_parameter("initialImuMinSamples", 50);
        get_parameter("initialImuMinSamples", initialImuMinSamples);
        declare_parameter("initialImuMinDuration", 0.5);
        get_parameter("initialImuMinDuration", initialImuMinDuration);
        declare_parameter("useImuPreintegrationInitialGuess", true);
        get_parameter("useImuPreintegrationInitialGuess", useImuPreintegrationInitialGuess);
        declare_parameter("useImuTranslationInitialGuess", false);
        get_parameter("useImuTranslationInitialGuess", useImuTranslationInitialGuess);
        declare_parameter("useImuRotationInitialGuess", true);
        get_parameter("useImuRotationInitialGuess", useImuRotationInitialGuess);
        declare_parameter("deskewMode", "imu_angular");
        get_parameter("deskewMode", deskewMode);
        declare_parameter("maxPointOffsetTimeSec", 0.2);
        get_parameter("maxPointOffsetTimeSec", maxPointOffsetTimeSec);

        if (imuType != "six_axis" && imuType != "nine_axis")
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Invalid imuType: " << imuType << " (use six_axis or nine_axis)");
            rclcpp::shutdown();
        }
        if (imuAccelerationUnit != "g" && imuAccelerationUnit != "mps2")
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Invalid imuAccelerationUnit: " << imuAccelerationUnit << " (use g or mps2)");
            rclcpp::shutdown();
        }
        if (deskewMode != "imu_angular" && deskewMode != "odom_interpolation" && deskewMode != "off")
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Invalid deskewMode: " << deskewMode << " (use imu_angular, odom_interpolation, or off)");
            rclcpp::shutdown();
        }
        if (imuAccelerationScale <= 0.0 || imuFrequency <= 0.0 || initialImuExpectedAccelerationNorm <= 0.0 ||
            initialImuMinSamples <= 0 || initialImuMinDuration < 0.0)
        {
            RCLCPP_ERROR(get_logger(), "Invalid IMU initialization or scaling parameter.");
            rclcpp::shutdown();
        }
        // AI_SHIP_ROBOT_END

        double ida[] = { 1.0,  0.0,  0.0,
                         0.0,  1.0,  0.0,
                         0.0,  0.0,  1.0};
        std::vector < double > id(ida, std::end(ida));
        declare_parameter("extrinsicRot", id);
        get_parameter("extrinsicRot", extRotV);
        declare_parameter("extrinsicRPY", id);
        get_parameter("extrinsicRPY", extRPYV);
        double zea[] = {0.0, 0.0, 0.0};
        std::vector < double > ze(zea, std::end(zea));
        declare_parameter("extrinsicTrans", ze);
        get_parameter("extrinsicTrans", extTransV);

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        declare_parameter("edgeThreshold", 1.0);
        get_parameter("edgeThreshold", edgeThreshold);
        declare_parameter("surfThreshold", 0.1);
        get_parameter("surfThreshold", surfThreshold);
        declare_parameter("edgeFeatureMinValidNum", 10);
        get_parameter("edgeFeatureMinValidNum", edgeFeatureMinValidNum);
        declare_parameter("surfFeatureMinValidNum", 100);
        get_parameter("surfFeatureMinValidNum", surfFeatureMinValidNum);

        declare_parameter("odometrySurfLeafSize", 0.4);
        get_parameter("odometrySurfLeafSize", odometrySurfLeafSize);
        declare_parameter("mappingCornerLeafSize", 0.2);
        get_parameter("mappingCornerLeafSize", mappingCornerLeafSize);
        declare_parameter("mappingSurfLeafSize", 0.4);
        get_parameter("mappingSurfLeafSize", mappingSurfLeafSize);
        // AI_SHIP_ROBOT_BEGIN: 近傍詳細点群とSLAM用粗点群を合成したPCD map保存用hybrid点群の設定を読む。
        declare_parameter("hybridRegisteredCloudEnabled", true);
        get_parameter("hybridRegisteredCloudEnabled", hybridRegisteredCloudEnabled);
        declare_parameter("hybridRegisteredCloudRawNearRange", 3.0);
        get_parameter("hybridRegisteredCloudRawNearRange", hybridRegisteredCloudRawNearRange);
        declare_parameter("hybridRegisteredCloudRawNearLeafSize", 0.01);
        get_parameter("hybridRegisteredCloudRawNearLeafSize", hybridRegisteredCloudRawNearLeafSize);
        declare_parameter("hybridRegisteredCloudTopic", "/lio_sam/mapping/cloud_registered_hybrid");
        get_parameter("hybridRegisteredCloudTopic", hybridRegisteredCloudTopic);
        declare_parameter("publishDeskewedCloud", false);
        get_parameter("publishDeskewedCloud", publishDeskewedCloud);
        declare_parameter("publishFeatureClouds", false);
        get_parameter("publishFeatureClouds", publishFeatureClouds);
        declare_parameter("publishMapGlobalCloud", false);
        get_parameter("publishMapGlobalCloud", publishMapGlobalCloud);
        declare_parameter("publishMapLocalCloud", false);
        get_parameter("publishMapLocalCloud", publishMapLocalCloud);
        declare_parameter("publishTrajectoryCloud", false);
        get_parameter("publishTrajectoryCloud", publishTrajectoryCloud);
        declare_parameter("publishCloudRegistered", false);
        get_parameter("publishCloudRegistered", publishCloudRegistered);
        declare_parameter("publishCloudRegisteredRaw", false);
        get_parameter("publishCloudRegisteredRaw", publishCloudRegisteredRaw);
        declare_parameter("publishLoopClosureClouds", false);
        get_parameter("publishLoopClosureClouds", publishLoopClosureClouds);
        if (hybridRegisteredCloudRawNearRange < 0.0 || hybridRegisteredCloudRawNearLeafSize < 0.0)
        {
            RCLCPP_ERROR(get_logger(), "Invalid hybrid registered cloud range or leaf size: must be non-negative.");
            rclcpp::shutdown();
        }
        // AI_SHIP_ROBOT_END

        declare_parameter("z_tollerance", 1000.0);
        get_parameter("z_tollerance", z_tollerance);
        declare_parameter("rotation_tollerance", 1000.0);
        get_parameter("rotation_tollerance", rotation_tollerance);

        declare_parameter("numberOfCores", 4);
        get_parameter("numberOfCores", numberOfCores);
        declare_parameter("mappingProcessInterval", 0.15);
        get_parameter("mappingProcessInterval", mappingProcessInterval);
        // AI_SHIP_ROBOT_BEGIN: backlog時に複数scanを連続処理し、重い処理の内訳を一定周期で出力する。
        declare_parameter("imageProjectionMaxScansPerTimer", 5);
        get_parameter("imageProjectionMaxScansPerTimer", imageProjectionMaxScansPerTimer);
        declare_parameter("imageProjectionBacklogLogThreshold", 10);
        get_parameter("imageProjectionBacklogLogThreshold", imageProjectionBacklogLogThreshold);
        declare_parameter("processingTimeLogIntervalSec", 5.0);
        get_parameter("processingTimeLogIntervalSec", processingTimeLogIntervalSec);
        if (imageProjectionMaxScansPerTimer <= 0 || imageProjectionBacklogLogThreshold < 0 || processingTimeLogIntervalSec < 0.0)
        {
            RCLCPP_ERROR(get_logger(), "Invalid processing performance parameter.");
            rclcpp::shutdown();
        }
        // AI_SHIP_ROBOT_END

        declare_parameter("surroundingkeyframeAddingDistThreshold", 1.0);
        get_parameter("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);
        declare_parameter("surroundingkeyframeAddingAngleThreshold", 0.2);
        get_parameter("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);
        declare_parameter("surroundingKeyframeDensity", 2.0);
        get_parameter("surroundingKeyframeDensity", surroundingKeyframeDensity);
        declare_parameter("surroundingKeyframeSearchRadius", 50.0);
        get_parameter("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);

        declare_parameter("loopClosureEnableFlag", true);
        get_parameter("loopClosureEnableFlag", loopClosureEnableFlag);
        declare_parameter("loopClosureFrequency", 1.0);
        get_parameter("loopClosureFrequency", loopClosureFrequency);
        declare_parameter("surroundingKeyframeSize", 50);
        get_parameter("surroundingKeyframeSize", surroundingKeyframeSize);
        declare_parameter("historyKeyframeSearchRadius", 15.0);
        get_parameter("historyKeyframeSearchRadius", historyKeyframeSearchRadius);
        declare_parameter("historyKeyframeSearchTimeDiff", 30.0);
        get_parameter("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
        declare_parameter("historyKeyframeSearchNum", 25);
        get_parameter("historyKeyframeSearchNum", historyKeyframeSearchNum);
        declare_parameter("historyKeyframeFitnessScore", 0.3);
        get_parameter("historyKeyframeFitnessScore", historyKeyframeFitnessScore);

        declare_parameter("globalMapVisualizationSearchRadius", 1000.0);
        get_parameter("globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
        declare_parameter("globalMapVisualizationPoseDensity", 10.0);
        get_parameter("globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
        declare_parameter("globalMapVisualizationLeafSize", 1.0);
        get_parameter("globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);

        usleep(100);
    }

    // AI_SHIP_ROBOT_BEGIN: 6軸IMUでは静止区間からroll/pitchとgyro biasだけを推定する。
    bool usingSixAxisImu() const
    {
        return imuType == "six_axis";
    }

    bool sixAxisImuReady() const
    {
        return !usingSixAxisImu() || sixAxisInitialOrientationReady;
    }

    bool shouldWaitForSixAxisImuInitialization() const
    {
        return usingSixAxisImu() && waitForImuInitialization && !sixAxisInitialOrientationReady;
    }

    double fallbackImuDeltaTime() const
    {
        return 1.0 / std::max(imuFrequency, 1.0e-6);
    }

    double accelerationScaleToMps2() const
    {
        return imuAccelerationScale * (imuAccelerationUnit == "g" ? imuGravity : 1.0);
    }

    double imuStampToSec(const sensor_msgs::msg::Imu& imu_in) const
    {
        return static_cast<double>(imu_in.header.stamp.sec) + static_cast<double>(imu_in.header.stamp.nanosec) * 1.0e-9;
    }

    void resetSixAxisInitialAccumulator()
    {
        sixAxisAccSum.setZero();
        sixAxisGyroSum.setZero();
        sixAxisAccNormSum = 0.0;
        sixAxisAccNormSquareSum = 0.0;
        sixAxisFirstSampleTime = 0.0;
        sixAxisSampleCount = 0;
    }

    bool vectorFinite(const Eigen::Vector3d& value) const
    {
        return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
    }

    void updateSixAxisInitialEstimate(const sensor_msgs::msg::Imu& imu_in)
    {
        if (!usingSixAxisImu() || sixAxisInitialOrientationReady)
            return;

        Eigen::Vector3d rawAcc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        Eigen::Vector3d rawGyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        const double accNorm = rawAcc.norm();
        const double gyrNorm = rawGyr.norm();

        // 静止区間だけを初期化サンプルに使い、走行中の並進加速度を重力方向として誤認しない。
        if (!vectorFinite(rawAcc) || !vectorFinite(rawGyr) ||
            std::abs(accNorm - initialImuExpectedAccelerationNorm) > initialImuAccelerationNormTolerance ||
            gyrNorm > initialImuMaxAngularVelocity)
        {
            resetSixAxisInitialAccumulator();
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Waiting for stationary 6-axis IMU samples: acc_norm=%.6f gyro_norm=%.6f", accNorm, gyrNorm);
            return;
        }

        const double stamp = imuStampToSec(imu_in);
        if (sixAxisSampleCount == 0 || stamp < sixAxisFirstSampleTime)
        {
            resetSixAxisInitialAccumulator();
            sixAxisFirstSampleTime = stamp;
        }

        sixAxisAccSum += rawAcc;
        sixAxisGyroSum += rawGyr;
        sixAxisAccNormSum += accNorm;
        sixAxisAccNormSquareSum += accNorm * accNorm;
        ++sixAxisSampleCount;

        const double duration = stamp - sixAxisFirstSampleTime;
        if (sixAxisSampleCount < initialImuMinSamples || duration < initialImuMinDuration)
            return;

        const double count = static_cast<double>(sixAxisSampleCount);
        const Eigen::Vector3d accMean = sixAxisAccSum / count;
        const double horizontalNorm = std::hypot(accMean.y(), accMean.z());
        if (!std::isfinite(horizontalNorm) || horizontalNorm <= std::numeric_limits<double>::epsilon())
        {
            resetSixAxisInitialAccumulator();
            RCLCPP_WARN(get_logger(), "Initial 6-axis acceleration average is degenerate; retry orientation estimate.");
            return;
        }

        // 6軸IMUではyawは観測できないため、roll/pitchとgyro biasだけを初期値として保持する。
        sixAxisInitialRoll = std::atan2(accMean.y(), accMean.z());
        sixAxisInitialPitch = std::atan2(-accMean.x(), horizontalNorm);
        sixAxisInitialGyroBias = sixAxisGyroSum / count;
        sixAxisInitialOrientationReady = true;

        const double accNormMean = sixAxisAccNormSum / count;
        const double accNormVariance = std::max(0.0, sixAxisAccNormSquareSum / count - accNormMean * accNormMean);
        const double gravityResidual = std::abs(accNormMean - initialImuExpectedAccelerationNorm);
        RCLCPP_INFO(
            get_logger(),
            "Initialized 6-axis IMU: roll=%.6f pitch=%.6f gyro_bias=(%.6f, %.6f, %.6f) "
            "acc_norm_mean=%.6f acc_norm_std=%.6f gravity_residual=%.6f samples=%d duration=%.3f",
            sixAxisInitialRoll, sixAxisInitialPitch,
            sixAxisInitialGyroBias.x(), sixAxisInitialGyroBias.y(), sixAxisInitialGyroBias.z(),
            accNormMean, std::sqrt(accNormVariance), gravityResidual, sixAxisSampleCount, duration);
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;

        updateSixAxisInitialEstimate(imu_in);

        // 加速度単位を明示parameterでm/s^2へ正規化してから、LiDAR/IMU外部回転を適用する。
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc *= accelerationScaleToMps2();
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();

        // 初期静止区間から推定したgyro biasを6軸モードで差し引き、preintegration driftを診断しやすくする。
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        if (usingSixAxisImu() && sixAxisInitialOrientationReady)
            gyr -= sixAxisInitialGyroBias;
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        Eigen::Quaterniond q_from;
        if (usingSixAxisImu())
        {
            tf2::Quaternion q_initial;
            q_initial.setRPY(
                sixAxisInitialOrientationReady ? sixAxisInitialRoll : 0.0,
                sixAxisInitialOrientationReady ? sixAxisInitialPitch : 0.0,
                0.0);
            q_initial.normalize();
            q_from = Eigen::Quaterniond(q_initial.w(), q_initial.x(), q_initial.y(), q_initial.z());
        }
        else
        {
            q_from = Eigen::Quaterniond(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        }

        // 9軸モードだけ入力orientationを絶対姿勢として扱い、6軸モードでは初期roll/pitch基準に限定する。
        Eigen::Quaterniond q_final = q_from * extQRPY;
        q_final.normalize();
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (!usingSixAxisImu() &&
            sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a valid 9-axis IMU orientation or imuType=six_axis.");
            rclcpp::shutdown();
        }

        if (imuDebug)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "IMU converted: type=%s acc=(%.6f, %.6f, %.6f) gyro=(%.6f, %.6f, %.6f) ready=%d",
                imuType.c_str(),
                imu_out.linear_acceleration.x, imu_out.linear_acceleration.y, imu_out.linear_acceleration.z,
                imu_out.angular_velocity.x, imu_out.angular_velocity.y, imu_out.angular_velocity.z,
                sixAxisImuReady() ? 1 : 0);
        }

        return imu_out;
    }
    // AI_SHIP_ROBOT_END
};


sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double stamp2Sec(const T& stamp)
{
    return rclcpp::Time(stamp).seconds();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

rmw_qos_profile_t qos_profile{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile.history,
        qos_profile.depth
    ),
    qos_profile);

rmw_qos_profile_t qos_profile_imu{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    2000,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos_imu = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_imu.history,
        qos_profile_imu.depth
    ),
    qos_profile_imu);

// AI_SHIP_ROBOT_BEGIN: rosbag再生時のLiDAR欠落を避けるため、LIO-SAM側のLiDAR購読をreliable高depthにする。
rmw_qos_profile_t qos_profile_lidar{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    200,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos_lidar = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_lidar.history,
        qos_profile_lidar.depth
    ),
    qos_profile_lidar);
// AI_SHIP_ROBOT_END

#endif
