// ROS 2 fault injector: consumes ideal sensor topics and republishes corrupted
// measurements onto /sensors/* according to a configurable scenario (Sec. 11).
// Fully deterministic given a fixed `seed`.
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <cmath>
#include <random>

namespace ugv::fault {

namespace {
constexpr double kMetersPerDegLat = 111'320.0;
double stampSec(const builtin_interfaces::msg::Time& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}
}  // namespace

/// Injects noise / bias / outage / slip into IMU, GNSS and wheel-odometry.
class FaultInjectorNode : public rclcpp::Node {
public:
    FaultInjectorNode() : rclcpp::Node("fault_injector_node") {
        seed_ = static_cast<std::uint64_t>(declare_parameter("seed", 42));
        rng_.seed(seed_);

        // IMU faults.
        imu_accel_noise_ = declare_parameter("imu.accel_noise_std", 0.0);
        imu_gyro_noise_ = declare_parameter("imu.gyro_noise_std", 0.0);
        imu_accel_bias_ = declare_parameter("imu.accel_bias", 0.0);
        imu_gyro_bias_ = declare_parameter("imu.gyro_bias", 0.0);
        imu_bias_walk_ = declare_parameter("imu.bias_random_walk", 0.0);

        // GNSS faults.
        gnss_noise_ = declare_parameter("gnss.noise_std", 0.0);
        gnss_bias_n_ = declare_parameter("gnss.bias_north", 0.0);
        gnss_bias_e_ = declare_parameter("gnss.bias_east", 0.0);
        gnss_multipath_ = declare_parameter("gnss.multipath_amp", 0.0);
        gnss_outage_enabled_ = declare_parameter("gnss.outage.enabled", false);
        gnss_outage_start_ = declare_parameter("gnss.outage.start", 0.0);
        gnss_outage_duration_ = declare_parameter("gnss.outage.duration", 0.0);

        // Wheel-odometry faults.
        odo_noise_ = declare_parameter("odometry.noise_std", 0.0);
        odo_scale_ = declare_parameter("odometry.scale_factor", 1.0);
        slip_enabled_ = declare_parameter("odometry.slip.enabled", false);
        slip_start_ = declare_parameter("odometry.slip.start", 0.0);
        slip_duration_ = declare_parameter("odometry.slip.duration", 0.0);
        slip_factor_ = declare_parameter("odometry.slip.factor", 0.3);

        const auto qos = rclcpp::SensorDataQoS();
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", qos, [this](sensor_msgs::msg::Imu::SharedPtr m) { onImu(*m); });
        gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", qos, [this](sensor_msgs::msg::NavSatFix::SharedPtr m) { onGnss(*m); });
        wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/wheel/odometry", qos,
            [this](nav_msgs::msg::Odometry::SharedPtr m) { onWheel(*m); });

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/sensors/imu", qos);
        gnss_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/sensors/gnss", qos);
        wheel_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sensors/wheel_odom", qos);

        RCLCPP_INFO(get_logger(), "fault_injector_node ready (seed=%lu).",
                    static_cast<unsigned long>(seed_));
    }

private:
    double gauss(double sigma) { return sigma > 0.0 ? sigma * normal_(rng_) : 0.0; }

    void initClock(double t) {
        if (!t0_set_) {
            t0_ = t;
            t0_set_ = true;
        }
    }

    void onImu(sensor_msgs::msg::Imu m) {
        const double t = stampSec(m.header.stamp);
        initClock(t);
        imu_accel_bias_walk_ += gauss(imu_bias_walk_);
        imu_gyro_bias_walk_ += gauss(imu_bias_walk_);
        m.linear_acceleration.x += imu_accel_bias_ + imu_accel_bias_walk_ + gauss(imu_accel_noise_);
        m.linear_acceleration.y += gauss(imu_accel_noise_);
        m.linear_acceleration.z += gauss(imu_accel_noise_);
        m.angular_velocity.x += gauss(imu_gyro_noise_);
        m.angular_velocity.y += gauss(imu_gyro_noise_);
        m.angular_velocity.z += imu_gyro_bias_ + imu_gyro_bias_walk_ + gauss(imu_gyro_noise_);
        imu_pub_->publish(m);
    }

    void onGnss(sensor_msgs::msg::NavSatFix m) {
        const double t = stampSec(m.header.stamp);
        initClock(t);
        const double rel = t - t0_;
        if (gnss_outage_enabled_ && rel >= gnss_outage_start_ &&
            rel < gnss_outage_start_ + gnss_outage_duration_) {
            return;  // drop the fix entirely during an outage
        }
        // Slowly-varying multipath-like bias + constant bias + white noise, in metres.
        const double mp = gnss_multipath_ * std::sin(0.1 * rel);
        const double dn = gnss_bias_n_ + mp + gauss(gnss_noise_);
        const double de = gnss_bias_e_ + mp + gauss(gnss_noise_);
        const double lat = m.latitude;
        m.latitude += dn / kMetersPerDegLat;
        m.longitude += de / (kMetersPerDegLat * std::cos(lat * M_PI / 180.0));
        const double var = std::max(gnss_noise_ * gnss_noise_, 0.25);
        m.position_covariance = {var, 0, 0, 0, var, 0, 0, 0, 4.0 * var};
        m.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        gnss_pub_->publish(m);
    }

    void onWheel(nav_msgs::msg::Odometry m) {
        const double t = stampSec(m.header.stamp);
        initClock(t);
        const double rel = t - t0_;
        double scale = odo_scale_;
        if (slip_enabled_ && rel >= slip_start_ && rel < slip_start_ + slip_duration_) {
            scale *= slip_factor_;  // wheels spin but robot slips -> under-report
        }
        m.twist.twist.linear.x = m.twist.twist.linear.x * scale + gauss(odo_noise_);
        m.twist.twist.angular.z = m.twist.twist.angular.z * scale + gauss(odo_noise_);
        wheel_pub_->publish(m);
    }

    std::uint64_t seed_{42};
    std::mt19937_64 rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};

    double imu_accel_noise_{}, imu_gyro_noise_{}, imu_accel_bias_{}, imu_gyro_bias_{},
        imu_bias_walk_{};
    double imu_accel_bias_walk_{0.0}, imu_gyro_bias_walk_{0.0};
    double gnss_noise_{}, gnss_bias_n_{}, gnss_bias_e_{}, gnss_multipath_{};
    bool gnss_outage_enabled_{false};
    double gnss_outage_start_{}, gnss_outage_duration_{};
    double odo_noise_{}, odo_scale_{1.0};
    bool slip_enabled_{false};
    double slip_start_{}, slip_duration_{}, slip_factor_{0.3};

    bool t0_set_{false};
    double t0_{0.0};

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_pub_;
};

}  // namespace ugv::fault

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ugv::fault::FaultInjectorNode>());
    rclcpp::shutdown();
    return 0;
}
