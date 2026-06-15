#!/usr/bin/env python3
"""Apply AI Ship Robot Mid-360 6-axis patches to UV-Lab LIO-SAM."""

from __future__ import annotations

import sys
from pathlib import Path


MARKER = "AI_SHIP_ROBOT_MID360_6AXIS_PATCH"


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def write_text_if_changed(path: Path, old_text: str, new_text: str) -> bool:
    if new_text == old_text:
        return False
    path.write_text(new_text, encoding="utf-8")
    return True


def replace_once(text: str, old: str, new: str, path: Path, label: str) -> str:
    old_stripped = strip_trailing_whitespace(old)
    new_stripped = strip_trailing_whitespace(new)
    if new in text:
        return text
    if new_stripped in text:
        return text
    if old_stripped in text:
        return text.replace(old_stripped, new_stripped, 1)
    if label == "deskewInfo" and "Odometry interpolation is not available yet" in text:
        return text
    if label == "odom interpolation fields" and "odomStartAvailable" in text:
        return text
    if label == "updateInitialGuess" and "IMU initial guess delta" in text:
        return text
    if label == "empty cloud guard" and "Received an empty Livox CustomMsg; skip this scan." in text:
        return text
    if label in {
        "retry queued clouds after IMU",
        "retry queued clouds after odometry",
        "queued cloud processing",
    } and "void processCloudQueue()" in text:
        return text
    if label == "timer paced IMU callback" and "点群処理は専用timerに任せ" in text:
        return text
    if label == "timer paced odometry callback" and "void processCloudQueue()" in text and "processCloudQueue();\n    }\n\n    void cloudHandler" not in text:
        return text
    if label == "timer paced LiDAR callback" and "void processCloudQueue()" in text and "processCloudQueue();\n    }\n\n    void processCloudQueue" not in text:
        return text
    if label == "single scan cloud queue processing" and "点群変換中はlockを解放" in text:
        return text
    if label == "queued deskew status" and "DeskewStatus deskewInfo()" in text:
        return text
    if label == "wait for required odometry before scan publish" and "Waiting for IMU preintegration odometry before scan publish" in text:
        return text
    if label == "allow initial scan without odometry" and "Publish initial scan without IMU preintegration odometry" in text:
        return text
    if label == "wait for usable IMU before preintegration init" and "systemInitialized == true && stamp2Sec(imuQueOpt.front().header.stamp) > currentCorrectionTime" in text:
        return text
    if label == "allow initial preintegration correction" and "systemInitialized == true && stamp2Sec(imuQueOpt.front().header.stamp) > currentCorrectionTime" in text:
        return text
    if label == "skip zero-duration IMU preintegration correction" and "Skip IMU preintegration correction without integrated IMU data" in text:
        return text
    if label == "preintegration bias noise duration" and "sqrt(integratedImuDuration) * noiseModelBetweenBias" in text:
        return text
    if label == "catch initial IMU preintegration optimizer errors" and "Initial IMU preintegration optimization failed" in text:
        return text
    if label == "publish imu odometry after initial correction" and "初回補正後からIMU odometryをpublishできるようにする" in text:
        return text
    if label == "catch running IMU preintegration optimizer errors" and "IMU preintegration optimization failed; reset and continue" in text:
        return text
    if old not in text:
        raise RuntimeError(f"Could not find {label} in {path}")
    return text.replace(old, new_stripped, 1)


def replace_all(text: str, old: str, new: str, path: Path, label: str) -> str:
    old_stripped = strip_trailing_whitespace(old)
    new_stripped = strip_trailing_whitespace(new)
    if old not in text:
        if new in text or new_stripped in text:
            return text
        if old_stripped in text:
            return text.replace(old_stripped, new_stripped)
        raise RuntimeError(f"Could not find {label} in {path}")
    return text.replace(old, new_stripped)


def strip_trailing_whitespace(text: str) -> str:
    lines = text.splitlines()
    stripped = "\n".join(line.rstrip() for line in lines)
    return stripped + ("\n" if text.endswith("\n") else "")


def patch_utility(lio_sam_dir: Path) -> bool:
    path = lio_sam_dir / "include" / "lio_sam" / "utility.hpp"
    original = read_text(path)
    text = original

    text = text.replace(
        "    string mapFrame;\n    string lidarInitFrame;\n\n    // GPS Settings\n",
        "    string mapFrame;\n\n    // GPS Settings\n",
    )
    text = text.replace(
        "        declare_parameter(\"mapFrame\", \"map\");\n"
        "        get_parameter(\"mapFrame\", mapFrame);\n"
        "        declare_parameter(\"lidarInitFrame\", \"base_lidar_init\");\n"
        "        get_parameter(\"lidarInitFrame\", lidarInitFrame);\n\n"
        "        declare_parameter(\"useImuHeadingInitialization\", false);\n",
        "        declare_parameter(\"mapFrame\", \"map\");\n"
        "        get_parameter(\"mapFrame\", mapFrame);\n\n"
        "        declare_parameter(\"useImuHeadingInitialization\", false);\n",
    )

    # 旧install scriptが行っていた9軸orientation反映パッチを内包し、クリーンcloneにも適用できるようにする。
    text = text.replace(
        "Eigen::Quaterniond q_final = extQRPY; //q_from * extQRPY;",
        "Eigen::Quaterniond q_final = q_from * extQRPY;",
    )

    old_members = """    float imuGravity;\n    float imuRPYWeight;\n    vector<double> extRotV;\n"""
    new_members = """    float imuGravity;\n    float imuRPYWeight;\n    string imuType;\n    string imuAccelerationUnit;\n    double imuAccelerationScale;\n    double imuFrequency;\n    bool imuDebug;\n    bool waitForImuInitialization;\n    double initialImuExpectedAccelerationNorm;\n    double initialImuAccelerationNormTolerance;\n    double initialImuMaxAngularVelocity;\n    int initialImuMinSamples;\n    double initialImuMinDuration;\n    bool useImuPreintegrationInitialGuess;\n    bool useImuTranslationInitialGuess;\n    bool useImuRotationInitialGuess;\n    string deskewMode;\n    double maxPointOffsetTimeSec;\n    bool sixAxisInitialOrientationReady = false;\n    double sixAxisInitialRoll = 0.0;\n    double sixAxisInitialPitch = 0.0;\n    Eigen::Vector3d sixAxisInitialGyroBias = Eigen::Vector3d::Zero();\n    Eigen::Vector3d sixAxisAccSum = Eigen::Vector3d::Zero();\n    Eigen::Vector3d sixAxisGyroSum = Eigen::Vector3d::Zero();\n    double sixAxisAccNormSum = 0.0;\n    double sixAxisAccNormSquareSum = 0.0;\n    double sixAxisFirstSampleTime = 0.0;\n    int sixAxisSampleCount = 0;\n    vector<double> extRotV;\n"""
    text = replace_once(text, old_members, new_members, path, "IMU parameter members")

    old_params = """        declare_parameter("imuRPYWeight", 0.01);\n        get_parameter("imuRPYWeight", imuRPYWeight);\n\n        double ida[] = { 1.0,  0.0,  0.0,\n"""
    new_params = """        declare_parameter("imuRPYWeight", 0.01);\n        get_parameter("imuRPYWeight", imuRPYWeight);\n        declare_parameter("imuType", "six_axis");\n        get_parameter("imuType", imuType);\n        declare_parameter("imuAccelerationUnit", "g");\n        get_parameter("imuAccelerationUnit", imuAccelerationUnit);\n        declare_parameter("imuAccelerationScale", 1.0);\n        get_parameter("imuAccelerationScale", imuAccelerationScale);\n        declare_parameter("imuFrequency", 500.0);\n        get_parameter("imuFrequency", imuFrequency);\n        declare_parameter("imuDebug", false);\n        get_parameter("imuDebug", imuDebug);\n        declare_parameter("waitForImuInitialization", true);\n        get_parameter("waitForImuInitialization", waitForImuInitialization);\n        declare_parameter("initialImuExpectedAccelerationNorm", 1.0);\n        get_parameter("initialImuExpectedAccelerationNorm", initialImuExpectedAccelerationNorm);\n        declare_parameter("initialImuAccelerationNormTolerance", 0.35);\n        get_parameter("initialImuAccelerationNormTolerance", initialImuAccelerationNormTolerance);\n        declare_parameter("initialImuMaxAngularVelocity", 0.2);\n        get_parameter("initialImuMaxAngularVelocity", initialImuMaxAngularVelocity);\n        declare_parameter("initialImuMinSamples", 50);\n        get_parameter("initialImuMinSamples", initialImuMinSamples);\n        declare_parameter("initialImuMinDuration", 0.5);\n        get_parameter("initialImuMinDuration", initialImuMinDuration);\n        declare_parameter("useImuPreintegrationInitialGuess", true);\n        get_parameter("useImuPreintegrationInitialGuess", useImuPreintegrationInitialGuess);\n        declare_parameter("useImuTranslationInitialGuess", false);\n        get_parameter("useImuTranslationInitialGuess", useImuTranslationInitialGuess);\n        declare_parameter("useImuRotationInitialGuess", true);\n        get_parameter("useImuRotationInitialGuess", useImuRotationInitialGuess);\n        declare_parameter("deskewMode", "imu_angular");\n        get_parameter("deskewMode", deskewMode);\n        declare_parameter("maxPointOffsetTimeSec", 0.2);\n        get_parameter("maxPointOffsetTimeSec", maxPointOffsetTimeSec);\n\n        if (imuType != "six_axis" && imuType != "nine_axis")\n        {\n            RCLCPP_ERROR_STREAM(get_logger(), "Invalid imuType: " << imuType << " (use six_axis or nine_axis)");\n            rclcpp::shutdown();\n        }\n        if (imuAccelerationUnit != "g" && imuAccelerationUnit != "mps2")\n        {\n            RCLCPP_ERROR_STREAM(get_logger(), "Invalid imuAccelerationUnit: " << imuAccelerationUnit << " (use g or mps2)");\n            rclcpp::shutdown();\n        }\n        if (deskewMode != "imu_angular" && deskewMode != "odom_interpolation" && deskewMode != "off")\n        {\n            RCLCPP_ERROR_STREAM(get_logger(), "Invalid deskewMode: " << deskewMode << " (use imu_angular, odom_interpolation, or off)");\n            rclcpp::shutdown();\n        }\n        if (imuAccelerationScale <= 0.0 || imuFrequency <= 0.0 || initialImuExpectedAccelerationNorm <= 0.0 ||\n            initialImuMinSamples <= 0 || initialImuMinDuration < 0.0)\n        {\n            RCLCPP_ERROR(get_logger(), "Invalid IMU initialization or scaling parameter.");\n            rclcpp::shutdown();\n        }\n\n        double ida[] = { 1.0,  0.0,  0.0,\n"""
    text = replace_once(text, old_params, new_params, path, "IMU parameter declarations")

    old_converter = """    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)\n    {\n        sensor_msgs::msg::Imu imu_out = imu_in;\n        // rotate acceleration\n        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);\n        \n        acc*=imuGravity;\n\n        acc = extRot * acc;\n        imu_out.linear_acceleration.x = acc.x();\n        imu_out.linear_acceleration.y = acc.y();\n        imu_out.linear_acceleration.z = acc.z();\n        // rotate gyroscope\n        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);\n        gyr = extRot * gyr;\n        imu_out.angular_velocity.x = gyr.x();\n        imu_out.angular_velocity.y = gyr.y();\n        imu_out.angular_velocity.z = gyr.z();\n        // rotate roll pitch yaw\n        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);\n        Eigen::Quaterniond q_final = q_from * extQRPY;\n        q_final.normalize();\n        imu_out.orientation.x = q_final.x();\n        imu_out.orientation.y = q_final.y();\n        imu_out.orientation.z = q_final.z();\n        imu_out.orientation.w = q_final.w();\n\n        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)\n        {\n            RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a 9-axis IMU!");\n            rclcpp::shutdown();\n        }\n\n        return imu_out;\n    }\n"""
    new_converter = f"""    bool usingSixAxisImu() const\n    {{\n        return imuType == "six_axis";\n    }}\n\n    bool sixAxisImuReady() const\n    {{\n        return !usingSixAxisImu() || sixAxisInitialOrientationReady;\n    }}\n\n    bool shouldWaitForSixAxisImuInitialization() const\n    {{\n        return usingSixAxisImu() && waitForImuInitialization && !sixAxisInitialOrientationReady;\n    }}\n\n    double fallbackImuDeltaTime() const\n    {{\n        return 1.0 / std::max(imuFrequency, 1.0e-6);\n    }}\n\n    double accelerationScaleToMps2() const\n    {{\n        return imuAccelerationScale * (imuAccelerationUnit == "g" ? imuGravity : 1.0);\n    }}\n\n    double imuStampToSec(const sensor_msgs::msg::Imu& imu_in) const\n    {{\n        return static_cast<double>(imu_in.header.stamp.sec) + static_cast<double>(imu_in.header.stamp.nanosec) * 1.0e-9;\n    }}\n\n    void resetSixAxisInitialAccumulator()\n    {{\n        sixAxisAccSum.setZero();\n        sixAxisGyroSum.setZero();\n        sixAxisAccNormSum = 0.0;\n        sixAxisAccNormSquareSum = 0.0;\n        sixAxisFirstSampleTime = 0.0;\n        sixAxisSampleCount = 0;\n    }}\n\n    bool vectorFinite(const Eigen::Vector3d& value) const\n    {{\n        return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());\n    }}\n\n    void updateSixAxisInitialEstimate(const sensor_msgs::msg::Imu& imu_in)\n    {{\n        if (!usingSixAxisImu() || sixAxisInitialOrientationReady)\n            return;\n\n        Eigen::Vector3d rawAcc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);\n        Eigen::Vector3d rawGyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);\n        const double accNorm = rawAcc.norm();\n        const double gyrNorm = rawGyr.norm();\n\n        // 静止区間だけを初期化サンプルに使い、走行中の並進加速度を重力方向として誤認しない。\n        if (!vectorFinite(rawAcc) || !vectorFinite(rawGyr) ||\n            std::abs(accNorm - initialImuExpectedAccelerationNorm) > initialImuAccelerationNormTolerance ||\n            gyrNorm > initialImuMaxAngularVelocity)\n        {{\n            resetSixAxisInitialAccumulator();\n            RCLCPP_WARN_THROTTLE(\n                get_logger(), *get_clock(), 2000,\n                "Waiting for stationary 6-axis IMU samples: acc_norm=%.6f gyro_norm=%.6f", accNorm, gyrNorm);\n            return;\n        }}\n\n        const double stamp = imuStampToSec(imu_in);\n        if (sixAxisSampleCount == 0 || stamp < sixAxisFirstSampleTime)\n        {{\n            resetSixAxisInitialAccumulator();\n            sixAxisFirstSampleTime = stamp;\n        }}\n\n        sixAxisAccSum += rawAcc;\n        sixAxisGyroSum += rawGyr;\n        sixAxisAccNormSum += accNorm;\n        sixAxisAccNormSquareSum += accNorm * accNorm;\n        ++sixAxisSampleCount;\n\n        const double duration = stamp - sixAxisFirstSampleTime;\n        if (sixAxisSampleCount < initialImuMinSamples || duration < initialImuMinDuration)\n            return;\n\n        const double count = static_cast<double>(sixAxisSampleCount);\n        const Eigen::Vector3d accMean = sixAxisAccSum / count;\n        const double horizontalNorm = std::hypot(accMean.y(), accMean.z());\n        if (!std::isfinite(horizontalNorm) || horizontalNorm <= std::numeric_limits<double>::epsilon())\n        {{\n            resetSixAxisInitialAccumulator();\n            RCLCPP_WARN(get_logger(), "Initial 6-axis acceleration average is degenerate; retry orientation estimate.");\n            return;\n        }}\n\n        // 6軸IMUではyawは観測できないため、roll/pitchとgyro biasだけを初期値として保持する。\n        sixAxisInitialRoll = std::atan2(accMean.y(), accMean.z());\n        sixAxisInitialPitch = std::atan2(-accMean.x(), horizontalNorm);\n        sixAxisInitialGyroBias = sixAxisGyroSum / count;\n        sixAxisInitialOrientationReady = true;\n\n        const double accNormMean = sixAxisAccNormSum / count;\n        const double accNormVariance = std::max(0.0, sixAxisAccNormSquareSum / count - accNormMean * accNormMean);\n        const double gravityResidual = std::abs(accNormMean - initialImuExpectedAccelerationNorm);\n        RCLCPP_INFO(\n            get_logger(),\n            "Initialized 6-axis IMU: roll=%.6f pitch=%.6f gyro_bias=(%.6f, %.6f, %.6f) "\n            "acc_norm_mean=%.6f acc_norm_std=%.6f gravity_residual=%.6f samples=%d duration=%.3f",\n            sixAxisInitialRoll, sixAxisInitialPitch,\n            sixAxisInitialGyroBias.x(), sixAxisInitialGyroBias.y(), sixAxisInitialGyroBias.z(),\n            accNormMean, std::sqrt(accNormVariance), gravityResidual, sixAxisSampleCount, duration);\n    }}\n\n    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)\n    {{\n        sensor_msgs::msg::Imu imu_out = imu_in;\n\n        updateSixAxisInitialEstimate(imu_in);\n\n        // 加速度単位を明示parameterでm/s^2へ正規化してから、LiDAR/IMU外部回転を適用する。\n        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);\n        acc *= accelerationScaleToMps2();\n        acc = extRot * acc;\n        imu_out.linear_acceleration.x = acc.x();\n        imu_out.linear_acceleration.y = acc.y();\n        imu_out.linear_acceleration.z = acc.z();\n\n        // 初期静止区間から推定したgyro biasを6軸モードで差し引き、preintegration driftを診断しやすくする。\n        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);\n        if (usingSixAxisImu() && sixAxisInitialOrientationReady)\n            gyr -= sixAxisInitialGyroBias;\n        gyr = extRot * gyr;\n        imu_out.angular_velocity.x = gyr.x();\n        imu_out.angular_velocity.y = gyr.y();\n        imu_out.angular_velocity.z = gyr.z();\n\n        Eigen::Quaterniond q_from;\n        if (usingSixAxisImu())\n        {{\n            tf2::Quaternion q_initial;\n            q_initial.setRPY(\n                sixAxisInitialOrientationReady ? sixAxisInitialRoll : 0.0,\n                sixAxisInitialOrientationReady ? sixAxisInitialPitch : 0.0,\n                0.0);\n            q_initial.normalize();\n            q_from = Eigen::Quaterniond(q_initial.w(), q_initial.x(), q_initial.y(), q_initial.z());\n        }}\n        else\n        {{\n            q_from = Eigen::Quaterniond(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);\n        }}\n\n        // 9軸モードだけ入力orientationを絶対姿勢として扱い、6軸モードでは初期roll/pitch基準に限定する。\n        Eigen::Quaterniond q_final = q_from * extQRPY;\n        q_final.normalize();\n        imu_out.orientation.x = q_final.x();\n        imu_out.orientation.y = q_final.y();\n        imu_out.orientation.z = q_final.z();\n        imu_out.orientation.w = q_final.w();\n\n        if (!usingSixAxisImu() &&\n            sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)\n        {{\n            RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a valid 9-axis IMU orientation or imuType=six_axis.");\n            rclcpp::shutdown();\n        }}\n\n        if (imuDebug)\n        {{\n            RCLCPP_INFO_THROTTLE(\n                get_logger(), *get_clock(), 1000,\n                "IMU converted: type=%s acc=(%.6f, %.6f, %.6f) gyro=(%.6f, %.6f, %.6f) ready=%d",\n                imuType.c_str(),\n                imu_out.linear_acceleration.x, imu_out.linear_acceleration.y, imu_out.linear_acceleration.z,\n                imu_out.angular_velocity.x, imu_out.angular_velocity.y, imu_out.angular_velocity.z,\n                sixAxisImuReady() ? 1 : 0);\n        }}\n\n        return imu_out;\n    }}\n"""
    text = replace_once(
        text,
        """rmw_qos_profile_t qos_profile_lidar{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
""",
        """rmw_qos_profile_t qos_profile_lidar{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    200,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
""",
        path,
        "LiDAR QoS depth",
    )

    text = replace_once(text, old_converter, new_converter, path, "imuConverter block")
    text = strip_trailing_whitespace(text)
    return write_text_if_changed(path, original, text)


def patch_image_projection(lio_sam_dir: Path) -> bool:
    path = lio_sam_dir / "src" / "imageProjection.cpp"
    original = read_text(path)
    text = original

    text = replace_once(
        text,
        """#include "utility.hpp"
#include "lio_sam/msg/cloud_info.hpp"
""",
        """#include "utility.hpp"
#include "lio_sam/msg/cloud_info.hpp"

#include <chrono>
""",
        path,
        "chrono include for cloud queue timer",
    )

    old_fields = """    bool odomDeskewFlag;\n    float odomIncreX;\n    float odomIncreY;\n    float odomIncreZ;\n\n    lio_sam::msg::CloudInfo cloudInfo;\n"""
    new_fields = """    bool odomDeskewFlag;\n    bool odomStartAvailable;\n    bool initialCloudWithoutOdomPublished = false;\n    Eigen::Affine3f transStartOdom;\n    float odomIncreX;\n    float odomIncreY;\n    float odomIncreZ;\n\n    lio_sam::msg::CloudInfo cloudInfo;\n"""
    text = replace_once(text, old_fields, new_fields, path, "odom interpolation fields")

    text = replace_once(
        text,
        """        odomDeskewFlag = false;\n\n        for (int i = 0; i < queueLength; ++i)\n""",
        """        odomDeskewFlag = false;\n        odomStartAvailable = false;\n\n        for (int i = 0; i < queueLength; ++i)\n""",
        path,
        "reset odomStartAvailable",
    )

    text = replace_once(
        text,
        """    std::mutex imuLock;
    std::mutex odoLock;
""",
        """    std::mutex imuLock;
    std::mutex odoLock;
    std::mutex cloudLock;

    enum class DeskewStatus
    {
        Ready,
        Wait,
        Drop,
    };
""",
        path,
        "cloud queue lock and deskew status",
    )
    text = replace_once(
        text,
        """    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subLaserCloud;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
""",
        """    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subLaserCloud;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
    rclcpp::CallbackGroup::SharedPtr callbackGroupCloudQueue;
    rclcpp::TimerBase::SharedPtr cloudQueueTimer;
""",
        path,
        "cloud queue timer members",
    )
    text = replace_once(
        text,
        """    bool odomStartAvailable;
    Eigen::Affine3f transStartOdom;
""",
        """    bool odomStartAvailable;
    bool initialCloudWithoutOdomPublished = false;
    Eigen::Affine3f transStartOdom;
""",
        path,
        "initial cloud without odom flag",
    )
    text = replace_once(
        text,
        """        callbackGroupOdom = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto lidarOpt = rclcpp::SubscriptionOptions();
""",
        """        callbackGroupOdom = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupCloudQueue = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto lidarOpt = rclcpp::SubscriptionOptions();
""",
        path,
        "cloud queue timer callback group",
    )
    text = replace_once(
        text,
        """        subLaserCloud = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            pointCloudTopic, qos_lidar,
            std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1),
            lidarOpt);

        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
""",
        """        subLaserCloud = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            pointCloudTopic, qos_lidar,
            std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1),
            lidarOpt);
        cloudQueueTimer = create_wall_timer(
            std::chrono::milliseconds(5), std::bind(&ImageProjection::processCloudQueue, this), callbackGroupCloudQueue);

        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
""",
        path,
        "cloud queue processing timer",
    )

    old_move = """    void moveFromCustomMsg(livox_ros_driver2::msg::CustomMsg &Msg, pcl::PointCloud<PointXYZIRT> & cloud)\n    {\n        cloud.clear();\n        cloud.reserve(Msg.point_num);\n        PointXYZIRT point;\n\n        cloud.header.frame_id=Msg.header.frame_id;\n        cloud.header.stamp= (uint64_t)((Msg.header.stamp.sec*1e9 + Msg.header.stamp.nanosec)/1000) ;\n        // cloud.header.seq=Msg.header.seq;\n\n        for(uint i=0;i<Msg.point_num-1;i++)\n        {\n            point.x=Msg.points[i].x; \n            point.y=Msg.points[i].y; \n            point.z=Msg.points[i].z; \n            point.intensity=Msg.points[i].reflectivity; \n            point.tag=Msg.points[i].tag; \n            point.time=Msg.points[i].offset_time*1e-9; \n            point.ring=Msg.points[i].line; \n            cloud.push_back(point);\n        }\n    }\n"""
    new_move = """    void moveFromCustomMsg(livox_ros_driver2::msg::CustomMsg &Msg, pcl::PointCloud<PointXYZIRT> & cloud)\n    {\n        cloud.clear();\n        cloud.reserve(Msg.point_num);\n        PointXYZIRT point;\n        uint32_t minOffset = std::numeric_limits<uint32_t>::max();\n        uint32_t maxOffset = 0;\n        uint32_t lastOffset = 0;\n        bool nonMonotonicOffset = false;\n\n        cloud.header.frame_id=Msg.header.frame_id;\n        cloud.header.stamp= (uint64_t)((Msg.header.stamp.sec*1e9 + Msg.header.stamp.nanosec)/1000) ;\n        // cloud.header.seq=Msg.header.seq;\n\n        for(uint i=0;i<Msg.point_num;i++)\n        {\n            const uint32_t offset = Msg.points[i].offset_time;\n            if (i > 0 && offset < lastOffset)\n                nonMonotonicOffset = true;\n            lastOffset = offset;\n            minOffset = std::min(minOffset, offset);\n            maxOffset = std::max(maxOffset, offset);\n\n            point.x=Msg.points[i].x; \n            point.y=Msg.points[i].y; \n            point.z=Msg.points[i].z; \n            point.intensity=Msg.points[i].reflectivity; \n            point.tag=Msg.points[i].tag; \n            point.time=offset*1e-9; \n            point.ring=Msg.points[i].line; \n            cloud.push_back(point);\n        }\n\n        // MID360のoffset_timeはdeskew品質へ直結するため、単位違いやsimulation由来の異常範囲をログで検出する。\n        if (Msg.point_num > 0)\n        {\n            const double minOffsetSec = static_cast<double>(minOffset) * 1e-9;\n            const double maxOffsetSec = static_cast<double>(maxOffset) * 1e-9;\n            if (nonMonotonicOffset || maxOffsetSec < 0.0 || maxOffsetSec > maxPointOffsetTimeSec)\n            {\n                RCLCPP_WARN_THROTTLE(\n                    get_logger(), *get_clock(), 2000,\n                    "Unexpected Livox offset_time range: min=%.9f max=%.9f sec non_monotonic=%d",\n                    minOffsetSec, maxOffsetSec, nonMonotonicOffset ? 1 : 0);\n            }\n        }\n    }\n"""
    text = replace_once(text, old_move, new_move, path, "CustomMsg conversion")

    text = replace_once(
        text,
        """        timeScanCur = stamp2Sec(cloudHeader.stamp);\n        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;\n\n        // check dense flag\n""",
        """        if (laserCloudIn->empty())\n        {\n            RCLCPP_WARN(get_logger(), "Received an empty Livox CustomMsg; skip this scan.");\n            return false;\n        }\n\n        timeScanCur = stamp2Sec(cloudHeader.stamp);\n        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;\n\n        // check dense flag\n""",
        path,
        "empty cloud guard",
    )

    text = replace_once(
        text,
        """        timeScanCur = stamp2Sec(cloudHeader.stamp);
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
""",
        """        timeScanCur = stamp2Sec(cloudHeader.stamp);
        double maxPointTime = 0.0;
        for (const auto& point : laserCloudIn->points)
            maxPointTime = std::max(maxPointTime, static_cast<double>(point.time));
        timeScanEnd = timeScanCur + maxPointTime;
""",
        path,
        "scan end time from max point offset",
    )

    text = replace_once(
        text,
        """        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
""",
        """        {
            std::lock_guard<std::mutex> lock1(imuLock);
            imuQueue.push_back(thisImu);
        }

        processCloudQueue();
""",
        path,
        "retry queued clouds after IMU",
    )
    text = replace_once(
        text,
        """        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
""",
        """        {
            std::lock_guard<std::mutex> lock2(odoLock);
            odomQueue.push_back(*odometryMsg);
        }

        processCloudQueue();
""",
        path,
        "retry queued clouds after odometry",
    )
    text = replace_once(
        text,
        """    void cloudHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        publishClouds();

        resetParameters();
    }
""",
        """    void cloudHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg)
    {
        {
            std::lock_guard<std::mutex> lock(cloudLock);
            cloudQueue.push_back(*laserCloudMsg);
        }

        processCloudQueue();
    }

    void processCloudQueue()
    {
        std::lock_guard<std::mutex> lock(cloudLock);

        while (rclcpp::ok() && !cloudQueue.empty())
        {
            if (!cachePointCloud())
            {
                resetParameters();
                continue;
            }

            const DeskewStatus deskewStatus = deskewInfo();
            if (deskewStatus == DeskewStatus::Wait)
                return;
            if (deskewStatus == DeskewStatus::Drop)
            {
                // 必要な時刻のIMUが既に失われたscanだけを破棄し、後続scanの処理を止めない。
                cloudQueue.pop_front();
                resetParameters();
                continue;
            }

            projectPointCloud();

            cloudExtraction();

            publishClouds();

            cloudQueue.pop_front();
            resetParameters();
        }
    }
""",
        path,
        "queued cloud processing",
    )
    text = replace_once(
        text,
        """        }

        processCloudQueue();

        // debug IMU data
""",
        """        }

        // 点群処理は専用timerに任せ、IMU callbackを詰まらせない。
        // debug IMU data
""",
        path,
        "timer paced IMU callback",
    )
    text = replace_once(
        text,
        """        }

        processCloudQueue();
    }

    void cloudHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg)
""",
        """        }
    }

    void cloudHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg)
""",
        path,
        "timer paced odometry callback",
    )
    text = replace_once(
        text,
        """        {
            std::lock_guard<std::mutex> lock(cloudLock);
            cloudQueue.push_back(*laserCloudMsg);
        }

        processCloudQueue();
    }

    void processCloudQueue()
""",
        """        {
            std::lock_guard<std::mutex> lock(cloudLock);
            cloudQueue.push_back(*laserCloudMsg);
        }
    }

    void processCloudQueue()
""",
        path,
        "timer paced LiDAR callback",
    )
    text = text.replace(
        """    void processCloudQueue()
    {
        std::lock_guard<std::mutex> lock(cloudLock);

        while (rclcpp::ok() && !cloudQueue.empty())
        {
            if (!cachePointCloud())
            {
                resetParameters();
                continue;
            }

            const DeskewStatus deskewStatus = deskewInfo();
            if (deskewStatus == DeskewStatus::Wait)
                return;
            if (deskewStatus == DeskewStatus::Drop)
            {
                // 必要な時刻のIMUが既に失われたscanだけを破棄し、後続scanの処理を止めない。
                cloudQueue.pop_front();
                resetParameters();
                continue;
            }

            projectPointCloud();

            cloudExtraction();

            publishClouds();

            cloudQueue.pop_front();
            resetParameters();
        }
    }
""",
        """    void processCloudQueue()
    {
        if (!rclcpp::ok())
            return;

        {
            std::lock_guard<std::mutex> lock(cloudLock);
            if (cloudQueue.empty())
                return;
            // 点群変換中はlockを解放し、rosbag再生中のLiDAR callback取りこぼしを避ける。
            currentCloudMsg = cloudQueue.front();
        }

        if (!cachePointCloud())
        {
            resetParameters();
            return;
        }

        const DeskewStatus deskewStatus = deskewInfo();
        if (deskewStatus == DeskewStatus::Wait)
            return;
        if (deskewStatus == DeskewStatus::Drop)
        {
            // 必要な時刻のIMUが既に失われたscanだけを破棄し、後続scanの処理を止めない。
            std::lock_guard<std::mutex> lock(cloudLock);
            if (!cloudQueue.empty())
                cloudQueue.pop_front();
            resetParameters();
            return;
        }

        // 1回のtimerで1scanだけ処理し、後段featureExtraction/mapOptimizationの入力queueをburstで溢れさせない。
        projectPointCloud();
        cloudExtraction();
        publishClouds();

        {
            std::lock_guard<std::mutex> lock(cloudLock);
            if (!cloudQueue.empty())
                cloudQueue.pop_front();
        }
        resetParameters();
    }
""",
    )
    text = text.replace(
        """    void processCloudQueue()
    {
        std::lock_guard<std::mutex> lock(cloudLock);
        if (!rclcpp::ok() || cloudQueue.empty())
            return;

        if (!cachePointCloud())
        {
            resetParameters();
            return;
        }

        const DeskewStatus deskewStatus = deskewInfo();
        if (deskewStatus == DeskewStatus::Wait)
            return;
        if (deskewStatus == DeskewStatus::Drop)
        {
            // 必要な時刻のIMUが既に失われたscanだけを破棄し、後続scanの処理を止めない。
            cloudQueue.pop_front();
            resetParameters();
            return;
        }

        // 1回のtimerで1scanだけ処理し、後段featureExtraction/mapOptimizationの入力queueをburstで溢れさせない。
        projectPointCloud();
        cloudExtraction();
        publishClouds();

        cloudQueue.pop_front();
        resetParameters();
    }
""",
        """    void processCloudQueue()
    {
        if (!rclcpp::ok())
            return;

        {
            std::lock_guard<std::mutex> lock(cloudLock);
            if (cloudQueue.empty())
                return;
            // 点群変換中はlockを解放し、rosbag再生中のLiDAR callback取りこぼしを避ける。
            currentCloudMsg = cloudQueue.front();
        }

        if (!cachePointCloud())
        {
            resetParameters();
            return;
        }

        const DeskewStatus deskewStatus = deskewInfo();
        if (deskewStatus == DeskewStatus::Wait)
            return;
        if (deskewStatus == DeskewStatus::Drop)
        {
            // 必要な時刻のIMUが既に失われたscanだけを破棄し、後続scanの処理を止めない。
            std::lock_guard<std::mutex> lock(cloudLock);
            if (!cloudQueue.empty())
                cloudQueue.pop_front();
            resetParameters();
            return;
        }

        // 1回のtimerで1scanだけ処理し、後段featureExtraction/mapOptimizationの入力queueをburstで溢れさせない。
        projectPointCloud();
        cloudExtraction();
        publishClouds();

        {
            std::lock_guard<std::mutex> lock(cloudLock);
            if (!cloudQueue.empty())
                cloudQueue.pop_front();
        }
        resetParameters();
    }
""",
    )
    text = text.replace(
        """    bool cachePointCloud(const livox_ros_driver2::msg::CustomMsg::SharedPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
""",
        """    bool cachePointCloud()
    {
        if (cloudQueue.empty())
            return false;

        // processCloudQueueで取り出したscanだけを変換し、重い処理中のcallback取りこぼしを避ける。
""",
    )
    text = text.replace(
        """        // queue先頭scanだけを変換し、IMUが揃わない場合はpopせず次回callbackで再試行する。
        currentCloudMsg = cloudQueue.front();
""",
        """        // processCloudQueueで取り出したscanだけを変換し、重い処理中のcallback取りこぼしを避ける。
""",
    )
    text = replace_once(
        text,
        """            RCLCPP_WARN(get_logger(), "Received an empty Livox CustomMsg; skip this scan.");
            return false;
""",
        """            RCLCPP_WARN(get_logger(), "Received an empty Livox CustomMsg; skip this scan.");
            cloudQueue.pop_front();
            return false;
""",
        path,
        "drop invalid empty cloud",
    )
    text = replace_once(
        text,
        """        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            "lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = create_publisher<lio_sam::msg::CloudInfo>(
            "lio_sam/deskew/cloud_info", qos);
""",
        """        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            "lio_sam/deskew/cloud_deskewed", 1);
        const auto cloudInfoQos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable().durability_volatile();
        pubLaserCloudInfo = create_publisher<lio_sam::msg::CloudInfo>(
            "lio_sam/deskew/cloud_info", cloudInfoQos);
""",
        path,
        "image projection CloudInfo QoS",
    )

    old_deskew_info = """    bool deskewInfo()\n    {\n        std::lock_guard<std::mutex> lock1(imuLock);\n        std::lock_guard<std::mutex> lock2(odoLock);\n\n        // make sure IMU data available for the scan\n        if (imuQueue.empty() ||\n            stamp2Sec(imuQueue.front().header.stamp) > timeScanCur ||\n            stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd)\n        {\n            RCLCPP_INFO(get_logger(), "Waiting for IMU data ...");\n            return false;\n        }\n\n        imuDeskewInfo();\n\n        odomDeskewInfo();\n\n        return true;\n    }\n"""
    new_deskew_info = """    bool deskewInfo()\n    {\n        std::lock_guard<std::mutex> lock1(imuLock);\n        std::lock_guard<std::mutex> lock2(odoLock);\n\n        if (shouldWaitForSixAxisImuInitialization())\n        {\n            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for initial 6-axis IMU roll/pitch estimate ...");\n            return false;\n        }\n\n        const bool needImuAngular = deskewMode == "imu_angular";\n        if (needImuAngular &&\n            (imuQueue.empty() || stamp2Sec(imuQueue.front().header.stamp) > timeScanCur ||\n             stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd))\n        {\n            RCLCPP_INFO(get_logger(), "Waiting for IMU data ...");\n            return false;\n        }\n\n        // deskewModeごとに必要な情報だけを要求し、odom補間deskewではIMU角速度依存を下げる。\n        if (!imuQueue.empty())\n            imuDeskewInfo();\n        else\n            cloudInfo.imu_available = false;\n\n        odomDeskewInfo();\n\n        if (deskewMode == "odom_interpolation" && !odomStartAvailable)\n        {\n            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odometry interpolation data ...");\n            return false;\n        }\n\n        return true;\n    }\n"""
    text = replace_once(text, old_deskew_info, new_deskew_info, path, "deskewInfo")
    text = text.replace(
        """        if (deskewMode == "odom_interpolation" && !odomStartAvailable)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odometry interpolation data ...");
            return false;
        }

        return true;
    }
""",
        """        if (deskewMode == "odom_interpolation" && !odomStartAvailable)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Odometry interpolation is not available yet; publish scan without odom deskew.");
        }

        return true;
    }
""",
    )

    text = replace_once(
        text,
        """    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        if (shouldWaitForSixAxisImuInitialization())
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for initial 6-axis IMU roll/pitch estimate ...");
            return false;
        }

        const bool needImuAngular = deskewMode == "imu_angular";
        if (needImuAngular &&
            (imuQueue.empty() || stamp2Sec(imuQueue.front().header.stamp) > timeScanCur ||
             stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd))
        {
            RCLCPP_INFO(get_logger(), "Waiting for IMU data ...");
            return false;
        }

        // deskewModeごとに必要な情報だけを要求し、odom補間deskewではIMU角速度依存を下げる。
        if (!imuQueue.empty())
            imuDeskewInfo();
        else
            cloudInfo.imu_available = false;

        odomDeskewInfo();

        if (deskewMode == "odom_interpolation" && !odomStartAvailable)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Odometry interpolation is not available yet; publish scan without odom deskew.");
        }

        return true;
    }
""",
        """    DeskewStatus deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        if (shouldWaitForSixAxisImuInitialization())
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for initial 6-axis IMU roll/pitch estimate ...");
            return DeskewStatus::Wait;
        }

        const bool needImuAngular = deskewMode == "imu_angular";
        if (needImuAngular)
        {
            if (imuQueue.empty() || stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd)
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for IMU data ...");
                return DeskewStatus::Wait;
            }
            if (stamp2Sec(imuQueue.front().header.stamp) > timeScanCur)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Drop scan because required start IMU is unavailable: scan_start=%.6f first_imu=%.6f",
                    timeScanCur, stamp2Sec(imuQueue.front().header.stamp));
                return DeskewStatus::Drop;
            }
        }

        // deskewModeごとに必要な情報だけを要求し、odom補間deskewではIMU角速度依存を下げる。
        if (!imuQueue.empty())
            imuDeskewInfo();
        else
            cloudInfo.imu_available = false;

        odomDeskewInfo();

        if (deskewMode == "odom_interpolation" && !odomStartAvailable)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Odometry interpolation is not available yet; publish scan without odom deskew.");
        }

        return DeskewStatus::Ready;
    }
""",
        path,
        "queued deskew status",
    )

    text = replace_once(
        text,
        """        odomDeskewInfo();

        if (deskewMode == "odom_interpolation" && !odomStartAvailable)
        {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Odometry interpolation is not available yet; publish scan without odom deskew.");
        }

        return DeskewStatus::Ready;
""",
        """        odomDeskewInfo();

        if (useImuPreintegrationInitialGuess && !cloudInfo.odom_available)
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
                RCLCPP_WARN(
                    get_logger(),
                    "Drop scan because required start odometry is unavailable: scan_start=%.6f first_odom=%.6f",
                    timeScanCur, stamp2Sec(odomQueue.front().header.stamp));
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
""",
        path,
        "wait for required odometry before scan publish",
    )

    text = replace_once(
        text,
        """        if (useImuPreintegrationInitialGuess && !cloudInfo.odom_available)
        {
            if (odomQueue.empty() || stamp2Sec(odomQueue.back().header.stamp) < timeScanCur)
            {
                RCLCPP_INFO_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "Waiting for IMU preintegration odometry before scan publish: scan_start=%.6f",
                    timeScanCur);
                return DeskewStatus::Wait;
            }
""",
        """        if (useImuPreintegrationInitialGuess && !cloudInfo.odom_available)
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
""",
        path,
        "allow initial scan without odometry",
    )

    old_odom = """    void odomDeskewInfo()\n    {\n        cloudInfo.odom_available = false;\n\n        while (!odomQueue.empty())\n        {\n            if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.01)\n                odomQueue.pop_front();\n            else\n                break;\n        }\n\n        if (odomQueue.empty())\n            return;\n\n        if (stamp2Sec(odomQueue.front().header.stamp) > timeScanCur)\n            return;\n\n        // get start odometry at the beinning of the scan\n        nav_msgs::msg::Odometry startOdomMsg;\n\n        for (int i = 0; i < (int)odomQueue.size(); ++i)\n        {\n            startOdomMsg = odomQueue[i];\n\n            if (stamp2Sec(startOdomMsg.header.stamp) < timeScanCur)\n                continue;\n            else\n                break;\n        }\n\n        tf2::Quaternion orientation;\n        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);\n\n        double roll, pitch, yaw;\n        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);\n\n        // Initial guess used in mapOptimization\n        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;\n        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;\n        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;\n        cloudInfo.initial_guess_roll = roll;\n        cloudInfo.initial_guess_pitch = pitch;\n        cloudInfo.initial_guess_yaw = yaw;\n\n        cloudInfo.odom_available = true;\n\n        // get end odometry at the end of the scan\n        odomDeskewFlag = false;\n\n        if (stamp2Sec(odomQueue.back().header.stamp) < timeScanEnd)\n            return;\n\n        nav_msgs::msg::Odometry endOdomMsg;\n\n        for (int i = 0; i < (int)odomQueue.size(); ++i)\n        {\n            endOdomMsg = odomQueue[i];\n\n            if (stamp2Sec(endOdomMsg.header.stamp) < timeScanEnd)\n                continue;\n            else\n                break;\n        }\n\n        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))\n            return;\n\n        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);\n\n        tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);\n        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);\n        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);\n\n        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;\n\n        float rollIncre, pitchIncre, yawIncre;\n        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);\n\n        odomDeskewFlag = true;\n    }\n"""
    new_odom = """    bool interpolateOdom(double targetTime, nav_msgs::msg::Odometry *odomOut)\n    {\n        if (odomQueue.empty() || stamp2Sec(odomQueue.front().header.stamp) > targetTime ||\n            stamp2Sec(odomQueue.back().header.stamp) < targetTime)\n            return false;\n\n        for (int i = 1; i < (int)odomQueue.size(); ++i)\n        {\n            const double backTime = stamp2Sec(odomQueue[i - 1].header.stamp);\n            const double frontTime = stamp2Sec(odomQueue[i].header.stamp);\n            if (frontTime < targetTime)\n                continue;\n\n            const nav_msgs::msg::Odometry& back = odomQueue[i - 1];\n            const nav_msgs::msg::Odometry& front = odomQueue[i];\n            const double duration = frontTime - backTime;\n            const double ratio = duration <= 0.0 ? 0.0 : (targetTime - backTime) / duration;\n\n            // 位置は線形補間、姿勢はslerpで補間し、scan内の任意時刻poseを作る。\n            *odomOut = front;\n            odomOut->pose.pose.position.x = back.pose.pose.position.x + ratio * (front.pose.pose.position.x - back.pose.pose.position.x);\n            odomOut->pose.pose.position.y = back.pose.pose.position.y + ratio * (front.pose.pose.position.y - back.pose.pose.position.y);\n            odomOut->pose.pose.position.z = back.pose.pose.position.z + ratio * (front.pose.pose.position.z - back.pose.pose.position.z);\n\n            tf2::Quaternion qBack, qFront;\n            tf2::fromMsg(back.pose.pose.orientation, qBack);\n            tf2::fromMsg(front.pose.pose.orientation, qFront);\n            tf2::Quaternion qInterpolated = qBack.slerp(qFront, ratio);\n            qInterpolated.normalize();\n            odomOut->pose.pose.orientation = tf2::toMsg(qInterpolated);\n            return true;\n        }\n\n        return false;\n    }\n\n    Eigen::Affine3f odomMsgToAffine(const nav_msgs::msg::Odometry& odomMsg)\n    {\n        tf2::Quaternion orientation;\n        tf2::fromMsg(odomMsg.pose.pose.orientation, orientation);\n        double roll, pitch, yaw;\n        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);\n        return pcl::getTransformation(\n            odomMsg.pose.pose.position.x, odomMsg.pose.pose.position.y, odomMsg.pose.pose.position.z,\n            roll, pitch, yaw);\n    }\n\n    void odomDeskewInfo()\n    {\n        cloudInfo.odom_available = false;\n        odomDeskewFlag = false;\n        odomStartAvailable = false;\n\n        while (!odomQueue.empty())\n        {\n            if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.2)\n                odomQueue.pop_front();\n            else\n                break;\n        }\n\n        nav_msgs::msg::Odometry startOdomMsg;\n        if (!interpolateOdom(timeScanCur, &startOdomMsg))\n            return;\n\n        tf2::Quaternion orientation;\n        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);\n        double roll, pitch, yaw;\n        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);\n\n        // mapOptimizationの初期値にはscan開始時刻へ補間したodomを渡し、キュー選択による時刻ずれを減らす。\n        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;\n        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;\n        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;\n        cloudInfo.initial_guess_roll = roll;\n        cloudInfo.initial_guess_pitch = pitch;\n        cloudInfo.initial_guess_yaw = yaw;\n        cloudInfo.odom_available = true;\n        odomStartAvailable = true;\n        transStartOdom = odomMsgToAffine(startOdomMsg);\n\n        nav_msgs::msg::Odometry endOdomMsg;\n        if (!interpolateOdom(timeScanEnd, &endOdomMsg))\n            return;\n\n        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))\n            return;\n\n        Eigen::Affine3f transBt = transStartOdom.inverse() * odomMsgToAffine(endOdomMsg);\n\n        float rollIncre, pitchIncre, yawIncre;\n        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);\n        odomDeskewFlag = true;\n    }\n"""
    text = replace_once(text, old_odom, new_odom, path, "odomDeskewInfo")
    text = text.replace(
        "        if (useImuPreintegrationInitialGuess && !cloudInfo.odom_available)\n",
        "        if (deskewMode != \"off\" && useImuPreintegrationInitialGuess && !cloudInfo.odom_available)\n",
    )

    old_find_position = """    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)\n    {\n        *posXCur = 0; *posYCur = 0; *posZCur = 0;\n\n        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.\n\n        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)\n        //     return;\n\n        // float ratio = relTime / (timeScanEnd - timeScanCur);\n\n        // *posXCur = ratio * odomIncreX;\n        // *posYCur = ratio * odomIncreY;\n        // *posZCur = ratio * odomIncreZ;\n    }\n\n    PointType deskewPoint(PointType *point, double relTime)\n    {\n        if (deskewFlag == -1 || cloudInfo.imu_available == false)\n            return *point;\n\n        double pointTime = timeScanCur + relTime;\n\n        float rotXCur, rotYCur, rotZCur;\n        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);\n\n        float posXCur, posYCur, posZCur;\n        findPosition(relTime, &posXCur, &posYCur, &posZCur);\n\n        if (firstPointFlag == true)\n        {\n            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();\n            firstPointFlag = false;\n        }\n\n        // transform points to start\n        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);\n        Eigen::Affine3f transBt = transStartInverse * transFinal;\n\n        PointType newPoint;\n        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);\n        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);\n        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);\n        newPoint.intensity = point->intensity;\n\n        return newPoint;\n    }\n"""
    new_find_position = """    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)\n    {\n        *posXCur = 0; *posYCur = 0; *posZCur = 0;\n\n        if (cloudInfo.odom_available == false || odomDeskewFlag == false)\n            return;\n\n        const double scanDuration = timeScanEnd - timeScanCur;\n        if (scanDuration <= 0.0)\n            return;\n\n        const float ratio = static_cast<float>(relTime / scanDuration);\n        *posXCur = ratio * odomIncreX;\n        *posYCur = ratio * odomIncreY;\n        *posZCur = ratio * odomIncreZ;\n    }\n\n    PointType applyDeskewTransform(PointType *point, const Eigen::Affine3f& transBt)\n    {\n        PointType newPoint;\n        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);\n        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);\n        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);\n        newPoint.intensity = point->intensity;\n        return newPoint;\n    }\n\n    PointType deskewPoint(PointType *point, double relTime)\n    {\n        if (deskewMode == "off")\n            return *point;\n\n        const double pointTime = timeScanCur + relTime;\n\n        if (deskewMode == "odom_interpolation")\n        {\n            nav_msgs::msg::Odometry pointOdomMsg;\n            if (!odomStartAvailable || !interpolateOdom(pointTime, &pointOdomMsg))\n                return *point;\n            return applyDeskewTransform(point, transStartOdom.inverse() * odomMsgToAffine(pointOdomMsg));\n        }\n\n        if (deskewFlag == -1 || cloudInfo.imu_available == false)\n            return *point;\n\n        float rotXCur, rotYCur, rotZCur;\n        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);\n\n        float posXCur, posYCur, posZCur;\n        findPosition(relTime, &posXCur, &posYCur, &posZCur);\n\n        if (firstPointFlag == true)\n        {\n            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();\n            firstPointFlag = false;\n        }\n\n        // IMU角速度積分deskewではscan開始から各点時刻までの相対姿勢へ点を戻す。\n        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);\n        return applyDeskewTransform(point, transStartInverse * transFinal);\n    }\n"""
    text = replace_once(text, old_find_position, new_find_position, path, "deskewPoint")
    text = strip_trailing_whitespace(text)
    return write_text_if_changed(path, original, text)


def patch_feature_extraction(lio_sam_dir: Path) -> bool:
    path = lio_sam_dir / "src" / "featureExtraction.cpp"
    original = read_text(path)
    text = original

    text = replace_once(
        text,
        """        subLaserCloudInfo = create_subscription<lio_sam::msg::CloudInfo>(
            "lio_sam/deskew/cloud_info", qos,
            std::bind(&FeatureExtraction::laserCloudInfoHandler, this, std::placeholders::_1));

        pubLaserCloudInfo = create_publisher<lio_sam::msg::CloudInfo>(
            "lio_sam/feature/cloud_info", qos);
""",
        """        const auto cloudInfoQos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable().durability_volatile();
        subLaserCloudInfo = create_subscription<lio_sam::msg::CloudInfo>(
            "lio_sam/deskew/cloud_info", cloudInfoQos,
            std::bind(&FeatureExtraction::laserCloudInfoHandler, this, std::placeholders::_1));

        pubLaserCloudInfo = create_publisher<lio_sam::msg::CloudInfo>(
            "lio_sam/feature/cloud_info", cloudInfoQos);
""",
        path,
        "feature extraction CloudInfo QoS",
    )

    text = strip_trailing_whitespace(text)
    return write_text_if_changed(path, original, text)


def patch_map_optimization(lio_sam_dir: Path) -> bool:
    path = lio_sam_dir / "src" / "mapOptmization.cpp"
    original = read_text(path)
    text = original
    text = text.replace(
        "    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;\n"
        "    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;\n"
        "    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncrementalInternal;\n",
        "    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;\n"
        "    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;\n",
    )
    text = text.replace(
        "        pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>(\n"
        "            \"lio_sam/mapping/odometry_incremental\", qos);\n"
        "        pubLaserOdometryIncrementalInternal = create_publisher<nav_msgs::msg::Odometry>(\n"
        "            \"lio_sam/mapping/odometry_incremental_internal\", qos);\n",
        "        pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>(\n"
        "            \"lio_sam/mapping/odometry_incremental\", qos);\n",
    )
    text = replace_once(
        text,
        """        subCloud = create_subscription<lio_sam::msg::CloudInfo>(
            "lio_sam/feature/cloud_info", qos,
            std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
""",
        """        const auto cloudInfoQos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable().durability_volatile();
        subCloud = create_subscription<lio_sam::msg::CloudInfo>(
            "lio_sam/feature/cloud_info", cloudInfoQos,
            std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
""",
        path,
        "map optimization CloudInfo QoS",
    )

    text = text.replace(
        "    float transformTobeMapped[6];\n"
        "    bool lidarOutputOriginInitialized = false;\n"
        "    Eigen::Affine3f lidarOutputOriginAffine = Eigen::Affine3f::Identity();\n",
        "    float transformTobeMapped[6];\n",
    )

    old_update = """    void updateInitialGuess()\n    {\n        // save current transformation before any processing\n        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);\n\n        static Eigen::Affine3f lastImuTransformation;\n        // initialization\n        if (cloudKeyPoses3D->points.empty())\n        {\n            transformTobeMapped[0] = cloudInfo.imu_roll_init;\n            transformTobeMapped[1] = cloudInfo.imu_pitch_init;\n            transformTobeMapped[2] = cloudInfo.imu_yaw_init;\n\n            if (!useImuHeadingInitialization)\n                transformTobeMapped[2] = 0;\n\n            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init); // save imu before return;\n            return;\n        }\n\n        // use imu pre-integration estimation for pose guess\n        static bool lastImuPreTransAvailable = false;\n        static Eigen::Affine3f lastImuPreTransformation;\n        if (cloudInfo.odom_available == true)\n        {\n            Eigen::Affine3f transBack = pcl::getTransformation(\n                cloudInfo.initial_guess_x, cloudInfo.initial_guess_y, cloudInfo.initial_guess_z,\n                cloudInfo.initial_guess_roll, cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);\n            if (lastImuPreTransAvailable == false)\n            {\n                lastImuPreTransformation = transBack;\n                lastImuPreTransAvailable = true;\n            } else {\n                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;\n                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);\n                Eigen::Affine3f transFinal = transTobe * transIncre;\n                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], \n                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);\n\n                lastImuPreTransformation = transBack;\n\n                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init); // save imu before return;\n                return;\n            }\n        }\n\n        // use imu incremental estimation for pose guess (only rotation)\n        if (cloudInfo.imu_available == true)\n        {\n            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);\n            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;\n\n            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);\n            Eigen::Affine3f transFinal = transTobe * transIncre;\n            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], \n                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);\n\n            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init); // save imu before return;\n            return;\n        }\n    }\n"""
    new_update = """    void updateInitialGuess()\n    {\n        // save current transformation before any processing\n        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);\n\n        static Eigen::Affine3f lastImuTransformation;\n        // initialization\n        if (cloudKeyPoses3D->points.empty())\n        {\n            transformTobeMapped[0] = cloudInfo.imu_roll_init;\n            transformTobeMapped[1] = cloudInfo.imu_pitch_init;\n            transformTobeMapped[2] = cloudInfo.imu_yaw_init;\n\n            if (!useImuHeadingInitialization)\n                transformTobeMapped[2] = 0;\n\n            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);\n            return;\n        }\n\n        static bool lastImuPreTransAvailable = false;\n        static Eigen::Affine3f lastImuPreTransformation;\n        if (cloudInfo.odom_available == true && useImuPreintegrationInitialGuess)\n        {\n            Eigen::Affine3f transBack = pcl::getTransformation(\n                cloudInfo.initial_guess_x, cloudInfo.initial_guess_y, cloudInfo.initial_guess_z,\n                cloudInfo.initial_guess_roll, cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);\n            if (lastImuPreTransAvailable == false)\n            {\n                lastImuPreTransformation = transBack;\n                lastImuPreTransAvailable = true;\n            }\n            else\n            {\n                Eigen::Affine3f transIncreRaw = lastImuPreTransformation.inverse() * transBack;\n                float xIncre, yIncre, zIncre, rollIncre, pitchIncre, yawIncre;\n                pcl::getTranslationAndEulerAngles(transIncreRaw, xIncre, yIncre, zIncre, rollIncre, pitchIncre, yawIncre);\n\n                // 6軸IMUではpreintegration並進が大きく壊れやすいため、並進・回転を独立に初期値へ採用する。\n                Eigen::Affine3f transIncre = pcl::getTransformation(\n                    useImuTranslationInitialGuess ? xIncre : 0.0f,\n                    useImuTranslationInitialGuess ? yIncre : 0.0f,\n                    useImuTranslationInitialGuess ? zIncre : 0.0f,\n                    useImuRotationInitialGuess ? rollIncre : 0.0f,\n                    useImuRotationInitialGuess ? pitchIncre : 0.0f,\n                    useImuRotationInitialGuess ? yawIncre : 0.0f);\n                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);\n                Eigen::Affine3f transFinal = transTobe * transIncre;\n                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],\n                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);\n\n                RCLCPP_INFO_THROTTLE(\n                    get_logger(), *get_clock(), 2000,\n                    "IMU initial guess delta: trans=(%.4f, %.4f, %.4f) norm=%.4f rot=(%.4f, %.4f, %.4f) use_trans=%d use_rot=%d",\n                    xIncre, yIncre, zIncre, std::sqrt(xIncre*xIncre + yIncre*yIncre + zIncre*zIncre),\n                    rollIncre, pitchIncre, yawIncre, useImuTranslationInitialGuess ? 1 : 0, useImuRotationInitialGuess ? 1 : 0);\n\n                lastImuPreTransformation = transBack;\n                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);\n                return;\n            }\n        }\n        else if (cloudInfo.odom_available == true)\n        {\n            lastImuPreTransformation = pcl::getTransformation(\n                cloudInfo.initial_guess_x, cloudInfo.initial_guess_y, cloudInfo.initial_guess_z,\n                cloudInfo.initial_guess_roll, cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);\n            lastImuPreTransAvailable = true;\n        }\n\n        // preintegration初期値を使わない場合でも、IMU角速度由来の相対回転だけはfallbackとして使えるようにする。\n        if (cloudInfo.imu_available == true)\n        {\n            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);\n            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;\n\n            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);\n            Eigen::Affine3f transFinal = transTobe * transIncre;\n            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],\n                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);\n\n            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);\n            return;\n        }\n    }\n"""
    text = replace_once(text, old_update, new_update, path, "updateInitialGuess")
    text = text.replace(
        "            if (!lidarOutputOriginInitialized)\n"
        "            {\n"
        "                // 公開odometry原点は最初のmapping出力ではなく、最初に処理したscanの初期姿勢へ固定する。\n"
        "                lidarOutputOriginAffine = trans2Affine3f(transformTobeMapped);\n"
        "                lidarOutputOriginInitialized = true;\n"
        "            }\n",
        "",
    )
    text = text.replace("pose_stamped.header.frame_id = lidarInitFrame;", "pose_stamped.header.frame_id = odometryFrame;")
    text = text.replace("laserOdometryROS.header.frame_id = lidarInitFrame;", "laserOdometryROS.header.frame_id = odometryFrame;")
    text = text.replace('laserOdometryROS.child_frame_id = "odom_mapping";', "laserOdometryROS.child_frame_id = lidarFrame;")
    text = text.replace("laserOdometryROS.child_frame_id = odometryFrame;", "laserOdometryROS.child_frame_id = lidarFrame;")
    text = text.replace(
        "tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point, lidarInitFrame);",
        "tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point, odometryFrame);",
    )
    text = text.replace('trans_odom_to_lidar.child_frame_id = "lidar_link";', "trans_odom_to_lidar.child_frame_id = lidarFrame;")
    text = text.replace("trans_odom_to_lidar.child_frame_id = odometryFrame;", "trans_odom_to_lidar.child_frame_id = lidarFrame;")
    text = text.replace(
        "        // AI_SHIP_ROBOT_MID360_6AXIS_PATCH: publish LiDAR odometry as base_lidar_init->odom.\n"
        "        br->sendTransform(trans_odom_to_lidar);",
        "        br->sendTransform(trans_odom_to_lidar);",
    )
    text = text.replace("laserOdomIncremental.header.frame_id = lidarInitFrame;", "laserOdomIncremental.header.frame_id = odometryFrame;")
    text = text.replace('laserOdomIncremental.child_frame_id = "odom_mapping";', "laserOdomIncremental.child_frame_id = lidarFrame;")
    text = text.replace("laserOdomIncremental.child_frame_id = odometryFrame;", "laserOdomIncremental.child_frame_id = lidarFrame;")

    text = text.replace(
        "\n    Eigen::Affine3f relativeLidarOutputAffine(const Eigen::Affine3f& internalPose)\n"
        "    {\n"
        "        // 内部推定で使う重力整列初期姿勢は保持し、外部公開時だけ初期LiDAR poseを差し引く。\n"
        "        if (!lidarOutputOriginInitialized)\n"
        "        {\n"
        "            lidarOutputOriginAffine = internalPose;\n"
        "            lidarOutputOriginInitialized = true;\n"
        "        }\n"
        "        return lidarOutputOriginAffine.inverse() * internalPose;\n"
        "    }\n"
        "\n"
        "    void setOdometryPoseFromAffine(nav_msgs::msg::Odometry& odometry, const Eigen::Affine3f& affine)\n"
        "    {\n"
        "        float x, y, z, roll, pitch, yaw;\n"
        "        pcl::getTranslationAndEulerAngles(affine, x, y, z, roll, pitch, yaw);\n"
        "\n"
        "        // 内部feedback用と公開用で同じ姿勢変換処理を使い、topic間の差をframe選択だけに限定する。\n"
        "        odometry.pose.pose.position.x = x;\n"
        "        odometry.pose.pose.position.y = y;\n"
        "        odometry.pose.pose.position.z = z;\n"
        "        tf2::Quaternion quat_tf;\n"
        "        quat_tf.setRPY(roll, pitch, yaw);\n"
        "        geometry_msgs::msg::Quaternion quat_msg;\n"
        "        tf2::convert(quat_tf, quat_msg);\n"
        "        odometry.pose.pose.orientation = quat_msg;\n"
        "    }\n",
        "",
    )

    text = text.replace(
        """    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = rclcpp::Time(pose_in.time * 1e9);
        pose_stamped.header.frame_id = odometryFrame;

        // path表示も公開odometryと同じ初期LiDAR相対座標へ変換する。
        Eigen::Affine3f outputPose = relativeLidarOutputAffine(pclPointToAffine3f(pose_in));
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(outputPose, x, y, z, roll, pitch, yaw);

        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }
""",
        """    void updatePath(const PointTypePose& pose_in)
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
""",
    )

    text = text.replace(
        """        Eigen::Affine3f internalLidarAffine = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f outputLidarAffine = relativeLidarOutputAffine(internalLidarAffine);
        float outputX, outputY, outputZ, outputRoll, outputPitch, outputYaw;
        pcl::getTranslationAndEulerAngles(outputLidarAffine, outputX, outputY, outputZ, outputRoll, outputPitch, outputYaw);

        laserOdometryROS.pose.pose.position.x = outputX;
        laserOdometryROS.pose.pose.position.y = outputY;
        laserOdometryROS.pose.pose.position.z = outputZ;
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(outputRoll, outputPitch, outputYaw);
""",
        """        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
""",
    )
    text = text.replace(
        """        quat_tf.setRPY(outputRoll, outputPitch, outputYaw);
        tf2::Transform t_odom_to_lidar = tf2::Transform(quat_tf, tf2::Vector3(outputX, outputY, outputZ));
""",
        """        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        tf2::Transform t_odom_to_lidar = tf2::Transform(quat_tf, tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
""",
    )
    text = text.replace(
        """            Eigen::Affine3f outputIncreOdomAffine = relativeLidarOutputAffine(increOdomAffine);
            pcl::getTranslationAndEulerAngles(outputIncreOdomAffine, x, y, z, roll, pitch, yaw);
""",
        """            pcl::getTranslationAndEulerAngles(increOdomAffine, x, y, z, roll, pitch, yaw);
""",
    )
    text = text.replace(
        "        static nav_msgs::msg::Odometry laserOdomIncremental; // incremental odometry msg\n"
        "        static nav_msgs::msg::Odometry laserOdomIncrementalInternal; // internal incremental odometry msg\n"
        "        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine\n",
        "        static nav_msgs::msg::Odometry laserOdomIncremental; // incremental odometry msg\n"
        "        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine\n",
    )
    text = text.replace(
        """            laserOdomIncremental = laserOdometryROS;
            laserOdomIncrementalInternal = laserOdometryROS;
            laserOdomIncrementalInternal.header.frame_id = odometryFrame;
            laserOdomIncrementalInternal.child_frame_id = "odom_mapping_internal";
            setOdometryPoseFromAffine(laserOdomIncrementalInternal, internalLidarAffine);
            increOdomAffine = internalLidarAffine;
""",
        """            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
""",
    )
    text = text.replace(
        """            // IMU preintegrationへ戻す内部topicは、重力整列済みの内部姿勢を保持する。
            laserOdomIncrementalInternal.header.stamp = timeLaserInfoStamp;
            laserOdomIncrementalInternal.header.frame_id = odometryFrame;
            laserOdomIncrementalInternal.child_frame_id = "odom_mapping_internal";
            setOdometryPoseFromAffine(laserOdomIncrementalInternal, increOdomAffine);
            if (isDegenerate)
                laserOdomIncrementalInternal.pose.covariance[0] = 1;
            else
                laserOdomIncrementalInternal.pose.covariance[0] = 0;

            // 外部公開topicだけは、初期LiDAR姿勢を差し引いた相対odometryとして出す。
""",
        "",
    )
    text = text.replace(
        "            // 公開用のLiDAR相対odometryには、内部IMU絶対roll/pitch補正を再注入しない。\n"
        "            if (false && cloudInfo.imu_available == true)\n",
        "            if (cloudInfo.imu_available == true)\n",
    )
    text = text.replace("        pubLaserOdometryIncrementalInternal->publish(laserOdomIncrementalInternal);\n", "")
    text = text.replace("globalPath.header.frame_id = lidarInitFrame;", "globalPath.header.frame_id = odometryFrame;")
    text = replace_once(text, 'double imuWeight = 0.1;', 'double imuWeight = imuRPYWeight;', path, "incremental imu RPY weight")
    text = strip_trailing_whitespace(text)
    return write_text_if_changed(path, original, text)


def patch_imu_preintegration(lio_sam_dir: Path) -> bool:
    path = lio_sam_dir / "src" / "imuPreintegration.cpp"
    original = read_text(path)
    text = original

    text = text.replace(
        "            \"lio_sam/mapping/odometry_incremental_internal\", qos,",
        "            \"lio_sam/mapping/odometry_incremental\", qos,",
    )
    text = text.replace("        tfBroadcaster->sendTransform(ts);", "        // AI_SHIP_ROBOT_MID360_6AXIS_PATCH: base_footprint is connected by static TF from left_lidar_odom.\n        // TransformFusion still publishes odometry topics, but does not publish a duplicate base TF.\n        // tfBroadcaster->sendTransform(ts);")

    text = text.replace(
        "        while (!imuOdomQueue.empty()) {\n"
        "            if (stamp2Sec(imuOdomQueue.front().header.stamp) <= lidarOdomTime)\n",
        "        // LiDAR補正時刻と同時刻のIMU odometryだけが残る場合でも、後続のfront/back参照を安全に保つ。\n"
        "        while (imuOdomQueue.size() > 1) {\n"
        "            if (stamp2Sec(imuOdomQueue.front().header.stamp) <= lidarOdomTime)\n",
    )

    text = replace_all(text, "(1.0 / 500.0)", "fallbackImuDeltaTime()", path, "IMU fallback dt")

    old_failure = """    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur) {\n        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());\n        if (vel.norm() > 30) {\n            RCLCPP_WARN(get_logger(), "Large velocity, reset IMU-preintegration!");\n            return true;\n        }\n\n        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());\n        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());\n        if (ba.norm() > 1.0 || bg.norm() > 1.0) {\n            RCLCPP_WARN(get_logger(), "Large bias, reset IMU-preintegration!");\n            return true;\n        }\n\n        return false;\n    }\n"""
    new_failure = """    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur) {\n        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());\n        const double velNorm = vel.norm();\n        if (velNorm > 30) {\n            RCLCPP_WARN(get_logger(), "Large velocity, reset IMU-preintegration! norm=%.6f vel=(%.6f, %.6f, %.6f)",\n                        velNorm, vel.x(), vel.y(), vel.z());\n            return true;\n        }\n\n        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());\n        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());\n        const double baNorm = ba.norm();\n        const double bgNorm = bg.norm();\n        if (baNorm > 1.0 || bgNorm > 1.0) {\n            RCLCPP_WARN(get_logger(), "Large bias, reset IMU-preintegration! acc_norm=%.6f gyr_norm=%.6f acc=(%.6f, %.6f, %.6f) gyr=(%.6f, %.6f, %.6f)",\n                        baNorm, bgNorm, ba.x(), ba.y(), ba.z(), bg.x(), bg.y(), bg.z());\n            return true;\n        }\n\n        return false;\n    }\n"""
    text = replace_once(text, old_failure, new_failure, path, "failureDetection")
    old_imu_queue_with_wait = """        sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);
        if (shouldWaitForSixAxisImuInitialization())
            return;

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);
"""
    old_imu_queue_without_wait = """        sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);
"""
    new_imu_queue = """        sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);

        // 初期化中のIMUも保持し、先に届いた初回mapping補正を捨てない。
        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);
        if (shouldWaitForSixAxisImuInitialization())
            return;
"""

    # upstreamのclean cloneと過去パッチ済み形状の両方を受け入れ、Docker build時の適用順差を吸収する。
    if new_imu_queue not in text:
        if old_imu_queue_with_wait in text:
            text = text.replace(old_imu_queue_with_wait, new_imu_queue, 1)
        else:
            text = replace_once(
                text,
                old_imu_queue_without_wait,
                new_imu_queue,
                path,
                "IMU initialization wait",
            )
    text = replace_once(
        text,
        """        // 0. initialize system
        if (systemInitialized == false) {
""",
        """        if (systemInitialized == true && stamp2Sec(imuQueOpt.front().header.stamp) > currentCorrectionTime)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for usable IMU before IMU preintegration correction: correction=%.6f first_imu=%.6f",
                currentCorrectionTime, stamp2Sec(imuQueOpt.front().header.stamp));
            return;
        }

        // 0. initialize system
        if (systemInitialized == false) {
""",
        path,
        "wait for usable IMU before preintegration init",
    )
    text = replace_once(
        text,
        """        if (stamp2Sec(imuQueOpt.front().header.stamp) > currentCorrectionTime)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for usable IMU before IMU preintegration correction: correction=%.6f first_imu=%.6f",
                currentCorrectionTime, stamp2Sec(imuQueOpt.front().header.stamp));
            return;
        }
""",
        """        if (systemInitialized == true && stamp2Sec(imuQueOpt.front().header.stamp) > currentCorrectionTime)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for usable IMU before IMU preintegration correction: correction=%.6f first_imu=%.6f",
                currentCorrectionTime, stamp2Sec(imuQueOpt.front().header.stamp));
            return;
        }
""",
        path,
        "allow initial preintegration correction",
    )
    unconditional_wait = """        if (stamp2Sec(imuQueOpt.front().header.stamp) > currentCorrectionTime)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Waiting for usable IMU before IMU preintegration correction: correction=%.6f first_imu=%.6f",
                currentCorrectionTime, stamp2Sec(imuQueOpt.front().header.stamp));
            return;
        }

"""
    if unconditional_wait in text and "systemInitialized == true && stamp2Sec(imuQueOpt.front().header.stamp) > currentCorrectionTime" in text:
        text = text.replace(unconditional_wait, "", 1)
    text = replace_once(
        text,
        """            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();
""",
        """            // optimize once
            try {
                optimizer.update(graphFactors, graphValues);
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "Initial IMU preintegration optimization failed; reset and wait for next correction: %s", e.what());
                resetOptimization();
                resetParams();
                return;
            }
            graphFactors.resize(0);
            graphValues.clear();
""",
        path,
        "catch initial IMU preintegration optimizer errors",
    )
    text = replace_once(
        text,
        """            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            key = 1;
            systemInitialized = true;
            return;
""",
        """            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            // 初回補正後からIMU odometryをpublishできるようにする。
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevStateOdom = prevState_;
            prevBiasOdom = prevBias_;
            lastImuT_imu = -1;
            doneFirstOpt = true;

            key = 1;
            systemInitialized = true;
            return;
""",
        path,
        "publish imu odometry after initial correction",
    )
    text = replace_once(
        text,
        """        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu =
            dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
""",
        """        const double integratedImuDuration = imuIntegratorOpt_->deltaTij();
        if (integratedImuDuration <= 1.0e-6)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Skip IMU preintegration correction without integrated IMU data: correction=%.6f",
                currentCorrectionTime);
            return;
        }

        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu =
            dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
""",
        path,
        "skip zero-duration IMU preintegration correction",
    )
    text = replace_once(
        text,
        "sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias",
        "sqrt(integratedImuDuration) * noiseModelBetweenBias",
        path,
        "preintegration bias noise duration",
    )
    text = replace_once(
        text,
        """        // optimize
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
""",
        """        // optimize
        try {
            optimizer.update(graphFactors, graphValues);
            optimizer.update();
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "IMU preintegration optimization failed; reset and continue: %s", e.what());
            graphFactors.resize(0);
            graphValues.clear();
            resetParams();
            return;
        }
        graphFactors.resize(0);
        graphValues.clear();
""",
        path,
        "catch running IMU preintegration optimizer errors",
    )
    text = strip_trailing_whitespace(text)
    return write_text_if_changed(path, original, text)


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: patch_lio_sam_mid360_6axis.py /path/to/lio_sam", file=sys.stderr)
        return 2

    lio_sam_dir = Path(sys.argv[1]).resolve()
    if not (lio_sam_dir / "package.xml").is_file():
        print(f"Invalid LIO-SAM directory: {lio_sam_dir}", file=sys.stderr)
        return 2

    changed = []
    for name, patcher in (
        ("utility.hpp", patch_utility),
        ("imageProjection.cpp", patch_image_projection),
        ("featureExtraction.cpp", patch_feature_extraction),
        ("mapOptmization.cpp", patch_map_optimization),
        ("imuPreintegration.cpp", patch_imu_preintegration),
    ):
        if patcher(lio_sam_dir):
            changed.append(name)

    if changed:
        print(f"Applied {MARKER}: {', '.join(changed)}")
    else:
        print(f"{MARKER} already applied")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
