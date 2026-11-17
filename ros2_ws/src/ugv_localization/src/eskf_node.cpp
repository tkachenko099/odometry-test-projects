// ROS 2 online ESKF node: fuses /sensors/{imu,gnss,wheel_odom} into a
// navigation solution using the ugv_nav_core Error-State Kalman Filter.
//
// Frame conventions. ROS (REP-103/105) uses an FLU body (x forward, y left,
// z up) and an ENU world; the filter core uses the aerospace FRD body and NED
// navigation frame. This node is the only place where the two meet, so every
// quantity is converted exactly once on the way in and once on the way out:
//   * IMU   : FLU -> FRD on input,
//   * GNSS  : geodetic -> local NED (core helper),
//   * output: NED/FRD state -> ENU pose + FLU body twist.
#include <ugv_nav_core/ugv_nav_core.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ugv::localization {

namespace api = ugv::nav::api;
using api::Geodetic;
using api::NavState;
using api::Quat;
using api::Vec3;

namespace frames {

/// Body FLU <-> FRD (a 180 deg roll; the map is its own inverse).
[[nodiscard]] inline Vec3 flipBody(const Vec3& v) { return {v.x(), -v.y(), -v.z()}; }

/// Navigation NED <-> ENU (swap the horizontal axes, negate vertical; also
/// its own inverse).
[[nodiscard]] inline Vec3 swapNav(const Vec3& v) { return {v.y(), v.x(), -v.z()}; }

/// Same NED <-> ENU map as a matrix, for rotating covariance blocks.
[[nodiscard]] inline Eigen::Matrix3d navSwapMatrix() {
    Eigen::Matrix3d t;
    t << 0.0, 1.0, 0.0,
         1.0, 0.0, 0.0,
         0.0, 0.0, -1.0;
    return t;
}

/// Attitude NED<-FRD (filter) -> ENU<-FLU (ROS):
/// q_enu_flu = q_enu_ned * q_ned_frd * q_frd_flu.
[[nodiscard]] inline Quat toRosAttitude(const Quat& q_ned_frd) {
    static const double h = std::sqrt(0.5);
    static const Quat q_enu_ned(0.0, h, h, 0.0);    // 180 deg about (1,1,0)/sqrt(2)
    static const Quat q_frd_flu(0.0, 1.0, 0.0, 0.0);  // 180 deg about x
    return (q_enu_ned * q_ned_frd * q_frd_flu).normalized();
}

/// Level attitude for a given ROS/ENU yaw (0 = East, CCW positive) expressed
/// as the filter's NED<-FRD quaternion (heading 0 = North, CW positive).
[[nodiscard]] inline Quat levelAttitudeFromEnuYaw(double yaw_enu) {
    return Quat(Eigen::AngleAxisd(M_PI / 2.0 - yaw_enu, Vec3::UnitZ()));
}

}  // namespace frames

/// Loosely-coupled GNSS/INS + wheel-odometry estimator (Section 13).
class EskfNode : public rclcpp::Node {
public:
    EskfNode() : rclcpp::Node("eskf_node") {
        // --- Parameters ---
        wheel_radius_ = declare_parameter("wheel_radius", 0.10);
        wheel_track_ = declare_parameter("wheel_track", 0.36);
        wheel_speed_std_ = declare_parameter("wheel_speed_std", 0.05);
        // Non-holonomic constraint (no side-slip / no hop); <= 0 disables it.
        nhc_std_ = declare_parameter("nhc_std", 0.1);
        nhc_vertical_std_ = declare_parameter("nhc_vertical_std", 0.3);
        frame_id_ = declare_parameter("frame_id", std::string{"map"});
        child_frame_ = declare_parameter("child_frame_id", std::string{"base_footprint"});

        api::EskfParams p;
        p.accel_noise = declare_parameter("accel_noise", p.accel_noise);
        p.gyro_noise = declare_parameter("gyro_noise", p.gyro_noise);
        p.accel_bias_walk = declare_parameter("accel_bias_walk", p.accel_bias_walk);
        p.gyro_bias_walk = declare_parameter("gyro_bias_walk", p.gyro_bias_walk);
        p.init_pos_std = declare_parameter("init_pos_std", p.init_pos_std);
        p.init_vel_std = declare_parameter("init_vel_std", p.init_vel_std);
        p.init_att_std_deg = declare_parameter("init_att_std_deg", p.init_att_std_deg);
        // Heading cannot be observed from position-only GNSS at rest, so it is
        // an explicit input: the initial ROS/ENU yaw (0 = East) plus how much to
        // trust it. A prior wider than `kUnknownHeadingStdDeg` means "unknown":
        // the node then waits for the vehicle to move and aligns the heading to
        // the GNSS course over ground before using wheel/NHC updates.
        init_yaw_enu_ = declare_parameter("init_yaw_deg", 0.0) * M_PI / 180.0;
        p.init_yaw_std_deg = declare_parameter("init_yaw_std_deg", 180.0);
        heading_aligned_ = p.init_yaw_std_deg < kUnknownHeadingStdDeg;
        align_distance_ = declare_parameter("align_distance", 3.0);
        // Optional map datum [lat_deg, lon_deg, alt_m]; empty = use the first fix.
        const auto datum = declare_parameter("datum", std::vector<double>{});
        if (datum.size() == 3U) {
            datum_ = Geodetic{datum[0] * M_PI / 180.0, datum[1] * M_PI / 180.0, datum[2]};
        } else if (!datum.empty()) {
            RCLCPP_WARN(get_logger(), "datum must be [lat_deg, lon_deg, alt_m]; ignoring it.");
        }
        align_min_speed_ = declare_parameter("align_min_speed", 0.3);
        filter_ = std::make_unique<api::Eskf>(p);

        // --- Sensor QoS: best-effort, keep-last(10) ---
        const auto qos = rclcpp::SensorDataQoS();
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/sensors/imu", qos, [this](sensor_msgs::msg::Imu::SharedPtr m) { onImu(*m); });
        gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/sensors/gnss", qos,
            [this](sensor_msgs::msg::NavSatFix::SharedPtr m) { onGnss(*m); });
        wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/sensors/wheel_odom", qos,
            [this](nav_msgs::msg::Odometry::SharedPtr m) { onWheel(*m); });

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/odom", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/localization/path", 10);
        RCLCPP_INFO(get_logger(), "eskf_node ready (reads /sensors/*).");
    }

private:
    static double stamp(const builtin_interfaces::msg::Time& t) {
        return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
    }

    void onImu(const sensor_msgs::msg::Imu& m) {
        const double t = stamp(m.header.stamp);
        // sensor_msgs/Imu is FLU; the filter integrates FRD specific force/rate.
        const Vec3 accel = frames::flipBody(
            {m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z});
        const Vec3 gyro = frames::flipBody(
            {m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z});
        last_gyro_frd_ = gyro;

        if (!initialised_) {
            last_imu_t_ = t;
            initialised_ = true;
            return;  // wait for a dt and a GNSS origin
        }
        const double dt = t - last_imu_t_;
        last_imu_t_ = t;
        if (dt <= 0.0 || dt > 1.0) return;  // reject gaps / reordering
        if (!origin_) return;               // hold until first GNSS fix defines NED origin

        filter_->predict(accel, gyro, t, dt);
        publish(m.header.stamp);
    }

    void onGnss(const sensor_msgs::msg::NavSatFix& m) {
        if (std::isnan(m.latitude) || std::isnan(m.longitude)) return;
        const Geodetic g{m.latitude * M_PI / 180.0, m.longitude * M_PI / 180.0, m.altitude};
        if (!origin_) {
            // A surveyed datum pins the local frame to the map; without one the
            // first fix becomes the origin and its noise becomes a constant
            // offset of the whole track.
            origin_ = datum_ ? *datum_ : g;
            NavState x0;  // level, at rest, at the first fix
            x0.p = api::llh2ned(g, *origin_);
            x0.q = frames::levelAttitudeFromEnuYaw(init_yaw_enu_);
            filter_->reset(x0);
            RCLCPP_INFO(get_logger(), "NED origin (%s) set to lat=%.7f lon=%.7f alt=%.2f",
                        datum_ ? "datum" : "first fix", origin_->lat * 180.0 / M_PI,
                        origin_->lon * 180.0 / M_PI, origin_->alt);
            return;
        }
        const Vec3 ned = api::llh2ned(g, *origin_);
        Vec3 std_ned(1.0, 1.0, 2.0);
        if (m.position_covariance_type !=
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
            std_ned = Vec3(std::sqrt(std::max(m.position_covariance[4], 1e-4)),  // ENU->N~E
                           std::sqrt(std::max(m.position_covariance[0], 1e-4)),
                           std::sqrt(std::max(m.position_covariance[8], 1e-4)));
        }
        filter_->updateGnssNed(ned, std_ned);
        if (!heading_aligned_) tryAlignHeading(ned, std_ned);
    }

    /// One-shot coarse alignment from the GNSS course over ground. The baseline
    /// starts where the vehicle begins to roll and must reach `align_distance`.
    void tryAlignHeading(const Vec3& ned, const Vec3& std_ned) {
        if (std::abs(last_wheel_speed_) < align_min_speed_ || !align_anchor_) {
            align_anchor_ = ned;  // not rolling (or first fix): restart the baseline
            return;
        }
        const double dn = ned.x() - align_anchor_->x();
        const double de = ned.y() - align_anchor_->y();
        const double dist = std::hypot(dn, de);
        if (dist < align_distance_) return;

        // Reversing travels opposite to where the nose points.
        const double course = std::atan2(de, dn) + (last_wheel_speed_ < 0.0 ? M_PI : 0.0);
        const double sigma_h = std::max(std_ned.x(), std_ned.y());
        const double sigma = std::atan2(std::sqrt(2.0) * sigma_h, dist);
        // Restart from the current fix rather than patching the heading in
        // place: with an open heading prior the GNSS updates so far were
        // linearised about a meaningless yaw and have dragged tilt and biases
        // along with it. A ground vehicle is level to within init_att_std.
        NavState x0;
        x0.p = ned;
        x0.q = Quat(Eigen::AngleAxisd(course, Vec3::UnitZ()));
        filter_->reset(x0);
        filter_->alignHeading(course, std::abs(last_wheel_speed_), sigma);
        heading_aligned_ = true;
        RCLCPP_INFO(get_logger(),
                    "heading aligned to GNSS course: %.1f deg from North (+/- %.1f deg, %.1f m baseline)",
                    course * 180.0 / M_PI, sigma * 180.0 / M_PI, dist);
    }

    void onWheel(const nav_msgs::msg::Odometry& m) {
        last_wheel_speed_ = m.twist.twist.linear.x;
        if (!origin_) return;
        // Both Jacobians are linearised about the heading, so they are only
        // valid once it is roughly known; until then run on IMU + GNSS alone.
        if (!heading_aligned_) return;
        filter_->updateWheelSpeed(last_wheel_speed_, wheel_speed_std_);
        // Applied at standstill too: together with the zero wheel speed it
        // acts as a zero-velocity update and stops the vertical channel from
        // integrating the start/stop impact spikes of the IMU.
        if (nhc_std_ > 0.0) filter_->updateNonHolonomic(nhc_std_, nhc_vertical_std_);
    }

    void publish(const builtin_interfaces::msg::Time& stamp) {
        const NavState x = filter_->state();

        // Pose in the ENU world frame, twist in the FLU child frame (REP-105).
        const Vec3 p_enu = frames::swapNav(x.p);
        const Quat q_ros = frames::toRosAttitude(x.q);
        const Eigen::Matrix3d R = x.q.toRotationMatrix();  // nav(NED) <- body(FRD)
        const Vec3 v_flu = frames::flipBody(R.transpose() * x.v);
        const Vec3 w_flu = frames::flipBody(last_gyro_frd_ - x.gyro_bias);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = frame_id_;
        odom.child_frame_id = child_frame_;
        odom.pose.pose.position.x = p_enu.x();
        odom.pose.pose.position.y = p_enu.y();
        odom.pose.pose.position.z = p_enu.z();
        odom.pose.pose.orientation.w = q_ros.w();
        odom.pose.pose.orientation.x = q_ros.x();
        odom.pose.pose.orientation.y = q_ros.y();
        odom.pose.pose.orientation.z = q_ros.z();
        odom.twist.twist.linear.x = v_flu.x();
        odom.twist.twist.linear.y = v_flu.y();
        odom.twist.twist.linear.z = v_flu.z();
        odom.twist.twist.angular.x = w_flu.x();
        odom.twist.twist.angular.y = w_flu.y();
        odom.twist.twist.angular.z = w_flu.z();

        // Rotate the position / attitude covariance blocks NED -> ENU. The
        // error-state layout is [pos(0..2), vel(3..5), att(6..8), ...] and the
        // attitude error is a nav-frame rotation vector, so both blocks rotate
        // with the same matrix.
        const auto P = filter_->covariance();
        const Eigen::Matrix3d T = frames::navSwapMatrix();
        constexpr int kPos = 0;
        constexpr int kAtt = 6;
        const Eigen::Matrix3d Ppp = T * P.block<3, 3>(kPos, kPos) * T.transpose();
        const Eigen::Matrix3d Ppa = T * P.block<3, 3>(kPos, kAtt) * T.transpose();
        const Eigen::Matrix3d Paa = T * P.block<3, 3>(kAtt, kAtt) * T.transpose();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                const auto r = static_cast<std::size_t>(i);
                const auto c = static_cast<std::size_t>(j);
                odom.pose.covariance[r * 6U + c] = Ppp(i, j);
                odom.pose.covariance[r * 6U + c + 3U] = Ppa(i, j);
                odom.pose.covariance[(r + 3U) * 6U + c] = Ppa(j, i);
                odom.pose.covariance[(r + 3U) * 6U + c + 3U] = Paa(i, j);
            }
        }
        odom_pub_->publish(odom);

        if (path_.poses.size() > 5000) path_.poses.clear();
        path_.header = odom.header;
        geometry_msgs::msg::PoseStamped ps;
        ps.header = odom.header;
        ps.pose = odom.pose.pose;
        path_.poses.push_back(ps);
        path_pub_->publish(path_);
    }

    std::unique_ptr<api::Eskf> filter_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;

    std::optional<Geodetic> origin_;
    std::optional<Geodetic> datum_;
    bool initialised_{false};
    double last_imu_t_{0.0};
    double init_yaw_enu_{0.0};
    Vec3 last_gyro_frd_{Vec3::Zero()};
    double wheel_radius_{0.10};
    double wheel_track_{0.36};
    double wheel_speed_std_{0.05};
    double nhc_std_{0.1};
    double nhc_vertical_std_{0.3};
    /// Initial heading priors at least this wide are treated as "unknown".
    static constexpr double kUnknownHeadingStdDeg = 45.0;
    bool heading_aligned_{true};
    double align_distance_{3.0};
    double align_min_speed_{0.3};
    double last_wheel_speed_{0.0};
    std::optional<Vec3> align_anchor_;
    std::string frame_id_{"map"};
    std::string child_frame_{"base_footprint"};
};

}  // namespace ugv::localization

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ugv::localization::EskfNode>());
    rclcpp::shutdown();
    return 0;
}
