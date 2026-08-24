// ROS 2 online ESKF node: fuses /sensors/{imu,gnss,wheel_odom} into a
// navigation solution using the ugv_nav_core Error-State Kalman Filter.
#include <ugv_nav_core/ugv_nav_core.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <cmath>
#include <memory>
#include <optional>

namespace ugv::localization {

namespace api = ugv::nav::api;
using api::Geodetic;
using api::NavState;
using api::Vec3;

/// Loosely-coupled GNSS/INS + wheel-odometry estimator (Section 13).
class EskfNode : public rclcpp::Node {
public:
    EskfNode() : rclcpp::Node("eskf_node") {
        // --- Parameters ---
        wheel_radius_ = declare_parameter("wheel_radius", 0.10);
        wheel_track_ = declare_parameter("wheel_track", 0.36);
        wheel_speed_std_ = declare_parameter("wheel_speed_std", 0.05);
        frame_id_ = declare_parameter("frame_id", std::string{"map"});
        child_frame_ = declare_parameter("child_frame_id", std::string{"base_footprint"});

        api::EskfParams p;
        p.accel_noise = declare_parameter("accel_noise", p.accel_noise);
        p.gyro_noise = declare_parameter("gyro_noise", p.gyro_noise);
        p.accel_bias_walk = declare_parameter("accel_bias_walk", p.accel_bias_walk);
        p.gyro_bias_walk = declare_parameter("gyro_bias_walk", p.gyro_bias_walk);
        p.init_pos_std = declare_parameter("init_pos_std", p.init_pos_std);
        p.init_vel_std = declare_parameter("init_vel_std", p.init_vel_std);
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
        const Vec3 accel(m.linear_acceleration.x, m.linear_acceleration.y,
                         m.linear_acceleration.z);
        const Vec3 gyro(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);

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
            origin_ = g;
            NavState x0;  // start at NED origin, level, at rest
            filter_->reset(x0);
            RCLCPP_INFO(get_logger(), "NED origin set to lat=%.7f lon=%.7f", m.latitude,
                        m.longitude);
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
    }

    void onWheel(const nav_msgs::msg::Odometry& m) {
        if (!origin_) return;
        filter_->updateWheelSpeed(m.twist.twist.linear.x, wheel_speed_std_);
    }

    void publish(const builtin_interfaces::msg::Time& stamp) {
        const NavState x = filter_->state();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = frame_id_;
        odom.child_frame_id = child_frame_;
        odom.pose.pose.position.x = x.p.x();
        odom.pose.pose.position.y = x.p.y();
        odom.pose.pose.position.z = x.p.z();
        odom.pose.pose.orientation.w = x.q.w();
        odom.pose.pose.orientation.x = x.q.x();
        odom.pose.pose.orientation.y = x.q.y();
        odom.pose.pose.orientation.z = x.q.z();
        odom.twist.twist.linear.x = x.v.x();
        odom.twist.twist.linear.y = x.v.y();
        odom.twist.twist.linear.z = x.v.z();
        const auto P = filter_->covariance();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) odom.pose.covariance[i * 6 + j] = P(i, j);
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
    bool initialised_{false};
    double last_imu_t_{0.0};
    double wheel_radius_{0.10};
    double wheel_track_{0.36};
    double wheel_speed_std_{0.05};
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
