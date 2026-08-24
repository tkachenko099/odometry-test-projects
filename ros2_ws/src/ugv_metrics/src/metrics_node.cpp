// ROS 2 metrics node: time-aligns estimated vs. ground-truth odometry, streams
// a CSV log and, on shutdown, computes the navigation metric report (Sec. 17)
// via ugv_nav_core and writes it as JSON.
#include <ugv_nav_core/ugv_nav_core.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace ugv::metrics {

namespace api = ugv::nav::api;
using api::PoseError;
using api::Quat;
using api::Vec3;

namespace {
double stampSec(const builtin_interfaces::msg::Time& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}
double yawOf(const geometry_msgs::msg::Quaternion& q) {
    return api::quat2euler(Quat(q.w, q.x, q.y, q.z)).z();
}
}  // namespace

/// Buffers ground-truth samples and pairs each estimate with the nearest one.
class MetricsNode : public rclcpp::Node {
public:
    MetricsNode() : rclcpp::Node("metrics_node") {
        output_csv_ = declare_parameter("output_csv", std::string{"data.csv"});
        output_json_ = declare_parameter("output_json", std::string{"metrics.json"});
        max_dt_ = declare_parameter("max_assoc_dt", 0.05);

        const auto qos = rclcpp::SensorDataQoS();
        gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/odom", qos,
            [this](nav_msgs::msg::Odometry::SharedPtr m) { onGt(*m); });
        est_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/localization/odom", qos,
            [this](nav_msgs::msg::Odometry::SharedPtr m) { onEst(*m); });

        csv_.open(output_csv_);
        csv_ << "t,gt_x,gt_y,gt_z,gt_yaw,gt_vx,est_x,est_y,est_z,est_yaw,est_vx\n";
        RCLCPP_INFO(get_logger(), "metrics_node logging to %s", output_csv_.c_str());
    }

    ~MetricsNode() override { finalise(); }

    void finalise() {
        if (finalised_) return;
        finalised_ = true;
        if (csv_.is_open()) csv_.close();
        const auto r = api::evaluate(samples_);
        nlohmann::json j{
            {"position_rmse", r.position_rmse}, {"ate", r.ate},
            {"rpe", r.rpe},                     {"velocity_rmse", r.velocity_rmse},
            {"yaw_rmse", r.yaw_rmse},           {"max_error", r.max_error},
            {"travelled_distance", r.travelled_distance},
            {"relative_drift_pct", r.relative_drift_pct},
            {"duration", r.duration},           {"samples", r.samples}};
        std::ofstream out(output_json_);
        out << j.dump(2) << '\n';
        RCLCPP_INFO(get_logger(),
                    "metrics: pos_rmse=%.3f ate=%.3f rpe=%.3f yaw_rmse=%.3f max=%.3f n=%zu",
                    r.position_rmse, r.ate, r.rpe, r.yaw_rmse, r.max_error, r.samples);
    }

private:
    void onGt(const nav_msgs::msg::Odometry& m) {
        gt_[stampSec(m.header.stamp)] = m;
        if (gt_.size() > 20000) gt_.erase(gt_.begin());
    }

    void onEst(const nav_msgs::msg::Odometry& m) {
        if (gt_.empty()) return;
        const double t = stampSec(m.header.stamp);
        auto it = gt_.lower_bound(t);
        if (it == gt_.end()) --it;
        if (it != gt_.begin()) {
            auto prev = std::prev(it);
            if (std::abs(prev->first - t) < std::abs(it->first - t)) it = prev;
        }
        if (std::abs(it->first - t) > max_dt_) return;
        const auto& gt = it->second;

        PoseError e;
        e.t = t;
        e.p_est = {m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z};
        e.p_gt = {gt.pose.pose.position.x, gt.pose.pose.position.y, gt.pose.pose.position.z};
        e.yaw_est = yawOf(m.pose.pose.orientation);
        e.yaw_gt = yawOf(gt.pose.pose.orientation);
        e.v_est = {m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z};
        e.v_gt = {gt.twist.twist.linear.x, gt.twist.twist.linear.y, gt.twist.twist.linear.z};
        samples_.push_back(e);

        csv_ << e.t << ',' << e.p_gt.x() << ',' << e.p_gt.y() << ',' << e.p_gt.z() << ','
             << e.yaw_gt << ',' << e.v_gt.x() << ',' << e.p_est.x() << ',' << e.p_est.y()
             << ',' << e.p_est.z() << ',' << e.yaw_est << ',' << e.v_est.x() << '\n';
    }

    std::string output_csv_, output_json_;
    double max_dt_{0.05};
    bool finalised_{false};
    std::ofstream csv_;
    std::map<double, nav_msgs::msg::Odometry> gt_;
    std::vector<PoseError> samples_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_, est_sub_;
};

}  // namespace ugv::metrics

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ugv::metrics::MetricsNode>();
    rclcpp::spin(node);
    node->finalise();
    rclcpp::shutdown();
    return 0;
}
