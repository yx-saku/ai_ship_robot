#include "utility.hpp"
#include "lio_sam/msg/cloud_info.hpp"

// AI_SHIP_ROBOT_BEGIN: LiDAR callbackを軽量化し、timerで1scanずつ処理するためにchronoを使う。
#include <chrono>
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
    // AI_SHIP_ROBOT_END

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subLaserCloud;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
    // AI_SHIP_ROBOT_BEGIN: 点群変換をcallbackから切り離すため、専用callback groupとtimerを使う。
    rclcpp::CallbackGroup::SharedPtr callbackGroupCloudQueue;
    rclcpp::TimerBase::SharedPtr cloudQueueTimer;
    // AI_SHIP_ROBOT_END
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<lio_sam::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    std::deque<livox_ros_driver2::msg::CustomMsg> cloudQueue;
    livox_ros_driver2::msg::CustomMsg currentCloudMsg;

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

    int deskewFlag;
    cv::Mat rangeMat;

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
            ParamServer("lio_sam_imageProjection", options), deskewFlag(0)
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
        subLaserCloud = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            pointCloudTopic, qos_lidar,
            std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1),
            lidarOpt);
        // AI_SHIP_ROBOT_BEGIN: 1回のtimerで1scanだけ処理し、後段queueのburstを避ける。
        cloudQueueTimer = create_wall_timer(
            std::chrono::milliseconds(5), std::bind(&ImageProjection::processCloudQueue, this), callbackGroupCloudQueue);
        // AI_SHIP_ROBOT_END

        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            "lio_sam/deskew/cloud_deskewed", 1);
        // rosbag再生や高頻度入力でCloudInfoが欠落しないよう、後段との接続だけreliable高depthにする。
        const auto cloudInfoQos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable().durability_volatile();
        pubLaserCloudInfo = create_publisher<lio_sam::msg::CloudInfo>(
            "lio_sam/deskew/cloud_info", cloudInfoQos);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

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

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
    {
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg);

        {
            std::lock_guard<std::mutex> lock1(imuLock);
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
    void cloudHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr laserCloudMsg)
    {
        {
            std::lock_guard<std::mutex> lock(cloudLock);
            cloudQueue.push_back(*laserCloudMsg);
        }
    }

    void processCloudQueue()
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
            // 空scanなど変換不能な入力はここで破棄し、queue操作の責務をtimer側へ集約する。
            std::lock_guard<std::mutex> lock(cloudLock);
            if (!cloudQueue.empty())
                cloudQueue.pop_front();
            resetParameters();
            return;
        }

        const DeskewStatus deskewStatus = deskewInfo();
        if (deskewStatus == DeskewStatus::Wait)
            return;
        if (deskewStatus == DeskewStatus::Drop)
        {
            // 必要な時刻のIMUやodomが既に失われたscanだけを破棄し、後続scanの処理を止めない。
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
    // AI_SHIP_ROBOT_END

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
            // 初期姿勢が未確定のscanを後で処理すると、古い点群に確定後の姿勢が適用され初期mapが歪む。
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Drop scan until initial 6-axis IMU roll/pitch estimate is ready ...");
            return DeskewStatus::Drop;
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
        int cloudSize = laserCloudIn->points.size();
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

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo->publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto IP = std::make_shared<ImageProjection>(options);
    exec.add_node(IP);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Image Projection Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
