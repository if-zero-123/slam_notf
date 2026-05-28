#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class FastlioCloudAlign
{
public:
    FastlioCloudAlign(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
    void spin();

private:
    enum class AlignMode
    {
        NONE,
        FIXED,
        IMU_AUTO
    };

    struct AttitudeRad
    {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    struct AttitudeDeg
    {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber cloudSub_;
    ros::Subscriber imuSub_;
    ros::Subscriber odomSub_;
    ros::Publisher alignedCloudPub_;
    mutable std::mutex stateMutex_;

    std::string cloudTopic_;
    std::string alignedCloudTopic_;
    std::string imuTopic_;
    std::string odomTopic_;
    std::string outputFrameId_;
    std::string alignModeName_;
    AlignMode alignMode_ = AlignMode::IMU_AUTO;

    tf2::Transform fixedWorldAlign_;
    tf2::Transform activeWorldAlign_;
    tf2::Vector3 originPosition_;
    tf2::Vector3 pendingOriginPosition_;

    bool zeroOrigin_ = true;
    bool printDebug_ = true;
    bool cloudReceived_ = false;
    bool imuReceived_ = false;
    bool odomReceived_ = false;
    bool alignReady_ = false;
    bool alignStatsActive_ = false;
    bool originInitialized_ = false;
    bool pendingOriginValid_ = false;
    bool waitingCloudWarned_ = false;
    bool waitingImuWarned_ = false;
    bool waitingOdomWarned_ = false;
    bool waitingAlignWarned_ = false;

    double alignDuration_ = 2.0;
    double maxAlignRpyStddevDeg_ = 3.0;
    int minAlignSamples_ = 50;

    ros::Time alignStartStamp_;
    int alignSampleCount_ = 0;
    int orientationSampleCount_ = 0;
    int accelSampleCount_ = 0;
    double sumRoll_ = 0.0;
    double sumPitch_ = 0.0;
    double sumRoll2_ = 0.0;
    double sumPitch2_ = 0.0;

    static tf2::Transform makeTransform(double x, double y, double z,
                                        double roll, double pitch, double yaw);
    static double radToDeg(double value);
    static double clampVariance(double value);
    static std::string modeToString(AlignMode mode);
    static AttitudeDeg toAttitudeDeg(const tf2::Quaternion &q);
    static void printLine();

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void resetImuAlignmentStats(const ros::Time &stamp);
    bool imuSampleToLevelRpy(const sensor_msgs::Imu &msg, AttitudeRad &rpy, std::string &source) const;
    void collectImuAlignmentSample(const sensor_msgs::Imu &msg, const ros::Time &stamp);
    bool tryLockImuAlignment(const ros::Time &stamp);
    bool ensureOriginReady();
    bool getCloudTransform(tf2::Transform &worldAlign, tf2::Vector3 &origin);
    bool transformCloud(const sensor_msgs::PointCloud2 &input,
                        sensor_msgs::PointCloud2 &output,
                        const tf2::Transform &worldAlign,
                        const tf2::Vector3 &origin) const;
    void printStartupSummary(double worldX, double worldY, double worldZ, const AttitudeDeg &fixedRpy) const;
    void printDebug(const sensor_msgs::PointCloud2 &output, int transformedCount, int skippedCount) const;
};

tf2::Transform FastlioCloudAlign::makeTransform(double x, double y, double z,
                                                double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    return tf2::Transform(q, tf2::Vector3(x, y, z));
}

double FastlioCloudAlign::radToDeg(double value)
{
    return value * 180.0 / M_PI;
}

double FastlioCloudAlign::clampVariance(double value)
{
    return std::max(0.0, value);
}

std::string FastlioCloudAlign::modeToString(AlignMode mode)
{
    switch (mode)
    {
    case AlignMode::NONE:
        return "none";
    case AlignMode::FIXED:
        return "fixed";
    case AlignMode::IMU_AUTO:
        return "imu_auto";
    }
    return "unknown";
}

FastlioCloudAlign::AttitudeDeg FastlioCloudAlign::toAttitudeDeg(const tf2::Quaternion &q)
{
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    AttitudeDeg attitude;
    attitude.roll = radToDeg(roll);
    attitude.pitch = radToDeg(pitch);
    attitude.yaw = radToDeg(yaw);
    return attitude;
}

void FastlioCloudAlign::printLine()
{
    ROS_INFO_STREAM("------------------------------------------------------------");
}

FastlioCloudAlign::FastlioCloudAlign(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh)
{
    double worldX = 0.0;
    double worldY = 0.0;
    double worldZ = 0.0;
    double worldRoll = 0.0;
    double worldPitch = 0.0;
    double worldYaw = 0.0;
    std::string alignMode;

    pnh_.param<std::string>("cloud_topic", cloudTopic_, "/cloud_registered");
    pnh_.param<std::string>("aligned_cloud_topic", alignedCloudTopic_, "/cloud_registered_aligned");
    pnh_.param<std::string>("imu_topic", imuTopic_, "/livox/imu");
    pnh_.param<std::string>("odom_topic", odomTopic_, "/Odom_high_freq");
    pnh_.param<std::string>("output_frame_id", outputFrameId_, "world");
    pnh_.param<std::string>("align_mode", alignMode, "imu_auto");

    pnh_.param<double>("world_x", worldX, 0.0);
    pnh_.param<double>("world_y", worldY, 0.0);
    pnh_.param<double>("world_z", worldZ, 0.0);
    pnh_.param<double>("world_roll", worldRoll, 0.0);
    pnh_.param<double>("world_pitch", worldPitch, 0.0);
    pnh_.param<double>("world_yaw", worldYaw, 0.0);

    pnh_.param<bool>("zero_origin", zeroOrigin_, true);
    pnh_.param<bool>("print_debug", printDebug_, true);
    pnh_.param<double>("startup_align_duration", alignDuration_, 2.0);
    pnh_.param<int>("min_align_samples", minAlignSamples_, 50);
    pnh_.param<double>("max_align_rpy_stddev_deg", maxAlignRpyStddevDeg_, 3.0);

    minAlignSamples_ = std::max(1, minAlignSamples_);
    alignDuration_ = std::max(0.1, alignDuration_);

    if (alignMode == "imu_auto" || alignMode == "imu" || alignMode == "auto")
    {
        alignMode_ = AlignMode::IMU_AUTO;
        alignReady_ = false;
    }
    else if (alignMode == "fixed" || alignMode == "manual")
    {
        alignMode_ = AlignMode::FIXED;
        alignReady_ = true;
    }
    else if (alignMode == "none" || alignMode == "raw")
    {
        alignMode_ = AlignMode::NONE;
        alignReady_ = true;
    }
    else
    {
        ROS_WARN_STREAM("fastlio_cloud_align: unknown align_mode='" << alignMode
                        << "', falling back to imu_auto");
        alignMode_ = AlignMode::IMU_AUTO;
        alignReady_ = false;
    }
    alignModeName_ = modeToString(alignMode_);

    fixedWorldAlign_ = makeTransform(worldX, worldY, worldZ, worldRoll, worldPitch, worldYaw);
    if (alignMode_ == AlignMode::FIXED)
    {
        activeWorldAlign_ = fixedWorldAlign_;
    }
    else
    {
        activeWorldAlign_.setIdentity();
    }

    cloudSub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        cloudTopic_, 2, &FastlioCloudAlign::cloudCallback, this);
    odomSub_ = nh_.subscribe<nav_msgs::Odometry>(
        odomTopic_, 20, &FastlioCloudAlign::odomCallback, this);
    if (alignMode_ == AlignMode::IMU_AUTO)
    {
        imuSub_ = nh_.subscribe<sensor_msgs::Imu>(
            imuTopic_, 100, &FastlioCloudAlign::imuCallback, this);
    }
    alignedCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>(alignedCloudTopic_, 2);

    printStartupSummary(worldX, worldY, worldZ, toAttitudeDeg(fixedWorldAlign_.getRotation()));

    if (alignMode_ == AlignMode::IMU_AUTO)
    {
        ROS_INFO_STREAM("Cloud alignment: waiting for " << imuTopic_
                        << ". Keep the vehicle static and level. Cloud output is blocked until lock.");
    }
    else if (alignMode_ == AlignMode::FIXED)
    {
        ROS_INFO_STREAM("Cloud alignment: fixed world rotation is active. IMU auto alignment is skipped.");
    }
    else
    {
        ROS_INFO_STREAM("Cloud alignment: disabled. FAST-LIO cloud coordinates are passed through with optional zero origin.");
    }
}

void FastlioCloudAlign::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    odomReceived_ = true;

    tf2::Vector3 rawPosition(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
    pendingOriginPosition_ = activeWorldAlign_ * rawPosition;
    pendingOriginValid_ = true;
}

void FastlioCloudAlign::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    imuReceived_ = true;

    if (alignMode_ != AlignMode::IMU_AUTO || alignReady_)
    {
        return;
    }

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    collectImuAlignmentSample(*msg, stamp);
}

void FastlioCloudAlign::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        cloudReceived_ = true;
        if (!alignReady_)
        {
            if (!waitingAlignWarned_)
            {
                ROS_INFO_STREAM("Cloud received, waiting for alignment before publishing " << alignedCloudTopic_);
                waitingAlignWarned_ = true;
            }
            return;
        }
    }

    tf2::Transform worldAlign;
    tf2::Vector3 origin;
    if (!getCloudTransform(worldAlign, origin))
    {
        return;
    }

    sensor_msgs::PointCloud2 output;
    if (!transformCloud(*msg, output, worldAlign, origin))
    {
        return;
    }

    alignedCloudPub_.publish(output);

    if (printDebug_)
    {
        printDebug(output, static_cast<int>(output.width * output.height), 0);
    }
}

void FastlioCloudAlign::resetImuAlignmentStats(const ros::Time &stamp)
{
    alignStatsActive_ = true;
    alignStartStamp_ = stamp;
    alignSampleCount_ = 0;
    orientationSampleCount_ = 0;
    accelSampleCount_ = 0;
    sumRoll_ = 0.0;
    sumPitch_ = 0.0;
    sumRoll2_ = 0.0;
    sumPitch2_ = 0.0;
}

bool FastlioCloudAlign::imuSampleToLevelRpy(const sensor_msgs::Imu &msg,
                                            AttitudeRad &rpy,
                                            std::string &source) const
{
    const geometry_msgs::Quaternion &qmsg = msg.orientation;
    const double qNorm2 = qmsg.x * qmsg.x + qmsg.y * qmsg.y + qmsg.z * qmsg.z + qmsg.w * qmsg.w;
    const bool covarianceAllowsOrientation = msg.orientation_covariance[0] != -1.0;
    const bool orientationFinite = std::isfinite(qmsg.x) && std::isfinite(qmsg.y) &&
                                   std::isfinite(qmsg.z) && std::isfinite(qmsg.w);

    if (covarianceAllowsOrientation && orientationFinite && qNorm2 > 1e-6)
    {
        tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
        q.normalize();
        double yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(rpy.roll, rpy.pitch, yaw);
        rpy.yaw = 0.0;
        source = "orientation";
        return true;
    }

    const geometry_msgs::Vector3 &acc = msg.linear_acceleration;
    const bool accFinite = std::isfinite(acc.x) && std::isfinite(acc.y) && std::isfinite(acc.z);
    const double accNorm = std::sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    if (accFinite && accNorm > 0.1)
    {
        rpy.roll = std::atan2(acc.y, acc.z);
        rpy.pitch = std::atan2(-acc.x, std::sqrt(acc.y * acc.y + acc.z * acc.z));
        rpy.yaw = 0.0;
        source = "linear_acceleration";
        return true;
    }

    return false;
}

void FastlioCloudAlign::collectImuAlignmentSample(const sensor_msgs::Imu &msg, const ros::Time &stamp)
{
    if (!alignStatsActive_ || stamp < alignStartStamp_)
    {
        resetImuAlignmentStats(stamp);
    }

    AttitudeRad sample;
    std::string source;
    if (!imuSampleToLevelRpy(msg, sample, source))
    {
        ROS_WARN_STREAM_THROTTLE(1.0, "fastlio_cloud_align: IMU sample has no valid orientation or acceleration");
        return;
    }

    sumRoll_ += sample.roll;
    sumPitch_ += sample.pitch;
    sumRoll2_ += sample.roll * sample.roll;
    sumPitch2_ += sample.pitch * sample.pitch;
    ++alignSampleCount_;
    if (source == "orientation")
    {
        ++orientationSampleCount_;
    }
    else
    {
        ++accelSampleCount_;
    }

    const double elapsed = (stamp - alignStartStamp_).toSec();
    const double progress = std::min(100.0, elapsed / alignDuration_ * 100.0);
    ROS_INFO_STREAM_THROTTLE(1.0, std::fixed << std::setprecision(2)
                                            << "Cloud align progress: " << progress << "%  "
                                            << elapsed << "/" << alignDuration_
                                            << "s  samples=" << alignSampleCount_
                                            << "  current IMU roll/pitch=["
                                            << radToDeg(sample.roll) << ", "
                                            << radToDeg(sample.pitch) << "] deg");

    if (elapsed >= alignDuration_ && alignSampleCount_ >= minAlignSamples_)
    {
        tryLockImuAlignment(stamp);
    }
}

bool FastlioCloudAlign::tryLockImuAlignment(const ros::Time &stamp)
{
    (void)stamp;
    const double n = static_cast<double>(alignSampleCount_);
    const double meanRoll = sumRoll_ / n;
    const double meanPitch = sumPitch_ / n;
    const double varRoll = clampVariance(sumRoll2_ / n - meanRoll * meanRoll);
    const double varPitch = clampVariance(sumPitch2_ / n - meanPitch * meanPitch);
    const double rollStddevDeg = radToDeg(std::sqrt(varRoll));
    const double pitchStddevDeg = radToDeg(std::sqrt(varPitch));

    if (rollStddevDeg > maxAlignRpyStddevDeg_ || pitchStddevDeg > maxAlignRpyStddevDeg_)
    {
        ROS_WARN_STREAM("Cloud alignment rejected: IMU roll/pitch is not stable enough. Retrying."
                        << " roll_std=" << rollStddevDeg
                        << " deg, pitch_std=" << pitchStddevDeg
                        << " deg, threshold=" << maxAlignRpyStddevDeg_ << " deg");
        resetImuAlignmentStats(ros::Time::now());
        return false;
    }

    tf2::Quaternion autoAlignQ;
    autoAlignQ.setRPY(meanRoll, meanPitch, 0.0);
    autoAlignQ.normalize();
    activeWorldAlign_ = tf2::Transform(autoAlignQ, tf2::Vector3(0.0, 0.0, 0.0));

    alignReady_ = true;
    alignStatsActive_ = false;
    originInitialized_ = false;
    pendingOriginValid_ = false;

    const AttitudeDeg alignRpy = toAttitudeDeg(autoAlignQ);
    printLine();
    ROS_INFO_STREAM("Cloud alignment locked");
    ROS_INFO_STREAM(std::fixed << std::setprecision(4)
                    << "IMU initial roll/pitch/yaw = ["
                    << radToDeg(meanRoll) << ", " << radToDeg(meanPitch) << ", 0.0000] deg");
    ROS_INFO_STREAM(std::fixed << std::setprecision(4)
                    << "Cloud world align roll/pitch/yaw = ["
                    << alignRpy.roll << ", " << alignRpy.pitch << ", " << alignRpy.yaw << "] deg");
    ROS_INFO_STREAM(std::fixed << std::setprecision(4)
                    << "samples=" << alignSampleCount_
                    << "  orientation=" << orientationSampleCount_
                    << "  acceleration=" << accelSampleCount_
                    << "  roll_std=" << rollStddevDeg
                    << " deg  pitch_std=" << pitchStddevDeg << " deg");
    ROS_INFO_STREAM("Cloud publishing starts after odom origin is available: " << alignedCloudTopic_);
    printLine();
    return true;
}

bool FastlioCloudAlign::ensureOriginReady()
{
    if (!zeroOrigin_)
    {
        if (!originInitialized_)
        {
            originPosition_.setValue(0.0, 0.0, 0.0);
            originInitialized_ = true;
        }
        return true;
    }

    if (originInitialized_)
    {
        return true;
    }

    if (!pendingOriginValid_)
    {
        ROS_INFO_STREAM_THROTTLE(1.0, "fastlio_cloud_align: waiting for odom origin from " << odomTopic_);
        return false;
    }

    originPosition_ = pendingOriginPosition_;
    originInitialized_ = true;
    ROS_INFO_STREAM(std::fixed << std::setprecision(4)
                    << "Cloud zero origin: origin xyz = ["
                    << originPosition_.x() << ", "
                    << originPosition_.y() << ", "
                    << originPosition_.z() << "]");
    return true;
}

bool FastlioCloudAlign::getCloudTransform(tf2::Transform &worldAlign, tf2::Vector3 &origin)
{
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (!ensureOriginReady())
    {
        return false;
    }

    worldAlign = activeWorldAlign_;
    origin = originPosition_;
    return true;
}

bool FastlioCloudAlign::transformCloud(const sensor_msgs::PointCloud2 &input,
                                       sensor_msgs::PointCloud2 &output,
                                       const tf2::Transform &worldAlign,
                                       const tf2::Vector3 &origin) const
{
    output = input;
    output.header.frame_id = outputFrameId_;
    output.header.stamp = input.header.stamp.isZero() ? ros::Time::now() : input.header.stamp;

    try
    {
        sensor_msgs::PointCloud2Iterator<float> iterX(output, "x");
        sensor_msgs::PointCloud2Iterator<float> iterY(output, "y");
        sensor_msgs::PointCloud2Iterator<float> iterZ(output, "z");

        for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ)
        {
            const float x = *iterX;
            const float y = *iterY;
            const float z = *iterZ;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            {
                continue;
            }

            const tf2::Vector3 rawPoint(static_cast<double>(x),
                                        static_cast<double>(y),
                                        static_cast<double>(z));
            const tf2::Vector3 alignedPoint = worldAlign * rawPoint - origin;
            *iterX = static_cast<float>(alignedPoint.x());
            *iterY = static_cast<float>(alignedPoint.y());
            *iterZ = static_cast<float>(alignedPoint.z());
        }
    }
    catch (const std::runtime_error &e)
    {
        ROS_ERROR_STREAM_THROTTLE(1.0, "fastlio_cloud_align: cannot transform cloud: " << e.what());
        return false;
    }

    return true;
}

void FastlioCloudAlign::printStartupSummary(double worldX, double worldY, double worldZ,
                                            const AttitudeDeg &fixedRpy) const
{
    printLine();
    ROS_INFO_STREAM("fastlio_cloud_align startup config");
    ROS_INFO_STREAM("mode: " << alignModeName_
                    << "    zero_origin=" << (zeroOrigin_ ? "true" : "false")
                    << "    print_debug=" << (printDebug_ ? "true" : "false"));
    ROS_INFO_STREAM("input: cloud=" << cloudTopic_
                    << "    imu=" << imuTopic_
                    << "    odom=" << odomTopic_);
    ROS_INFO_STREAM("output: " << alignedCloudTopic_
                    << "    frame_id=" << outputFrameId_);
    ROS_INFO_STREAM(std::fixed << std::setprecision(4)
                    << "fixed world align xyz=["
                    << worldX << ", " << worldY << ", " << worldZ
                    << "] m, rpy=["
                    << fixedRpy.roll << ", "
                    << fixedRpy.pitch << ", "
                    << fixedRpy.yaw << "] deg");
    if (alignMode_ == AlignMode::IMU_AUTO)
    {
        ROS_INFO_STREAM(std::fixed << std::setprecision(2)
                        << "IMU auto alignment: " << alignDuration_
                        << " s, min_samples=" << minAlignSamples_
                        << ", stddev_limit=" << maxAlignRpyStddevDeg_ << " deg");
    }
    printLine();
}

void FastlioCloudAlign::printDebug(const sensor_msgs::PointCloud2 &output,
                                   int transformedCount,
                                   int skippedCount) const
{
    static ros::Time lastPrintTime(0);
    const ros::Time now = ros::Time::now();
    if (!lastPrintTime.isZero() && (now - lastPrintTime).toSec() < 1.0)
    {
        return;
    }
    lastPrintTime = now;

    ROS_INFO_STREAM("fastlio_cloud_align publish "
                    << alignedCloudTopic_
                    << " points=" << transformedCount
                    << " skipped=" << skippedCount
                    << " frame=" << output.header.frame_id
                    << " zero_origin=" << (zeroOrigin_ ? "true" : "false"));
}

void FastlioCloudAlign::spin()
{
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Rate rate(10);
    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lock(stateMutex_);
            if (!cloudReceived_ && !waitingCloudWarned_)
            {
                ROS_WARN_STREAM("Waiting for cloud: " << cloudTopic_);
                waitingCloudWarned_ = true;
            }
            if (alignMode_ == AlignMode::IMU_AUTO && !imuReceived_ && !waitingImuWarned_)
            {
                ROS_WARN_STREAM("Waiting for imu: " << imuTopic_);
                waitingImuWarned_ = true;
            }
            if (zeroOrigin_ && !odomReceived_ && !waitingOdomWarned_)
            {
                ROS_WARN_STREAM("Waiting for odom origin: " << odomTopic_);
                waitingOdomWarned_ = true;
            }
        }
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fastlio_cloud_align");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    FastlioCloudAlign node(nh, pnh);
    node.spin();
    return 0;
}
