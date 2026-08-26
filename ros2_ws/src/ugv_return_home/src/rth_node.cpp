// ROS 2 return-home failsafe node.
//
// Bridges the ABI facade `ugv::rth::api::MissionController` to ROS topics and,
// through the injected `IAutopilotLink`, to an ArduPilot autopilot. Position and
// operator heartbeat come in from MAVLink (mavros); on link loss the node
// commands the autopilot back along the recorded trail.
//
// Two autopilot backends implement the same `IAutopilotLink` seam:
//   * RosBridgeLink  - mavros-agnostic: publishes the backtrack Path (RViz) and
//                      the commanded mode, and logs the exact MAVLink actions.
//   * MavrosLink     - real MAVLink via mavros services (SetMode / WaypointPush /
//                      WaypointClear). Compiled only when `mavros_msgs` is found
//                      (HAVE_MAVROS), e.g. for ArduPilot SITL / hardware-in-loop.
#include "ugv_return_home/ugv_return_home.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#ifdef HAVE_MAVROS
#include <mavros_msgs/msg/waypoint.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/waypoint_clear.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <span>
#include <string>

namespace ugv::rth {

constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kEarthR = 6'371'000.0;

[[nodiscard]] inline std::string modeName(api::AutopilotMode m) {
    switch (m) {
        case api::AutopilotMode::kManual: return "MANUAL";
        case api::AutopilotMode::kHold: return "HOLD";
        case api::AutopilotMode::kGuided: return "GUIDED";
        case api::AutopilotMode::kAuto: return "AUTO";
        case api::AutopilotMode::kReturnToLaunch: return "RTL";
        default: return "MANUAL";
    }
}
[[nodiscard]] inline std::string missionName(api::MissionMode m) {
    switch (m) {
        case api::MissionMode::kIdle: return "IDLE";
        case api::MissionMode::kRecording: return "RECORDING";
        case api::MissionMode::kReturning: return "RETURNING";
        case api::MissionMode::kHome: return "HOME";
        default: return "?";
    }
}

// --------------------------------------------------------------------------- //
// Backend 1: diagnostic ROS bridge (default; no mavros required)
// --------------------------------------------------------------------------- //
class RosBridgeLink final : public api::IAutopilotLink {
public:
    RosBridgeLink(rclcpp::Node& node, std::string frame_id)
        : node_(node), frame_id_(std::move(frame_id)) {
        // Latched (transient-local) so RViz / late subscribers get the last mission.
        const auto latched = rclcpp::QoS(1).transient_local();
        path_pub_ = node_.create_publisher<nav_msgs::msg::Path>("/rth/return_path", latched);
        mode_pub_ = node_.create_publisher<std_msgs::msg::String>("/rth/autopilot_mode", latched);
    }

    void setMode(api::AutopilotMode mode) override {
        std_msgs::msg::String msg;
        msg.data = modeName(mode);
        mode_pub_->publish(msg);
        RCLCPP_WARN(node_.get_logger(), "[MAVLink] SET_MODE -> %s", msg.data.c_str());
    }

    void uploadReturnPath(std::span<const api::Waypoint> path) override {
        RCLCPP_WARN(node_.get_logger(), "[MAVLink] uploading %zu-item backtrack mission",
                    path.size());
        if (path.empty()) return;
        const api::GeoPoint home = path.back().pos;  // last item is home
        nav_msgs::msg::Path msg;
        msg.header.stamp = node_.now();
        msg.header.frame_id = frame_id_;
        for (const api::Waypoint& wp : path) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = (wp.pos.lon - home.lon) * std::cos(home.lat) * kEarthR;  // East
            ps.pose.position.y = (wp.pos.lat - home.lat) * kEarthR;                        // North
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        path_pub_->publish(msg);
    }

private:
    rclcpp::Node& node_;
    std::string frame_id_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
};

#ifdef HAVE_MAVROS
// --------------------------------------------------------------------------- //
// Backend 2: real MAVLink via mavros (ArduPilot SITL / HIL)
// --------------------------------------------------------------------------- //
class MavrosLink final : public api::IAutopilotLink {
public:
    explicit MavrosLink(rclcpp::Node& node) : node_(node) {
        set_mode_ = node_.create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        push_ = node_.create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push");
        clear_ = node_.create_client<mavros_msgs::srv::WaypointClear>("/mavros/mission/clear");
    }

    void setMode(api::AutopilotMode mode) override {
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->base_mode = 0;
        req->custom_mode = modeName(mode);  // ArduRover mode names
        if (!set_mode_->wait_for_service(std::chrono::milliseconds(200))) {
            RCLCPP_ERROR(node_.get_logger(), "mavros /mavros/set_mode unavailable");
            return;
        }
        (void)set_mode_->async_send_request(req);
        RCLCPP_WARN(node_.get_logger(), "[mavros] SET_MODE -> %s", req->custom_mode.c_str());
    }

    void uploadReturnPath(std::span<const api::Waypoint> path) override {
        if (clear_->wait_for_service(std::chrono::milliseconds(200))) {
            (void)clear_->async_send_request(
                std::make_shared<mavros_msgs::srv::WaypointClear::Request>());
        }
        auto req = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
        req->start_index = 0;
        req->waypoints.reserve(path.size());
        bool first = true;
        for (const api::Waypoint& wp : path) {
            mavros_msgs::msg::Waypoint m;
            m.frame = mavros_msgs::msg::Waypoint::FRAME_GLOBAL;
            m.command = 16;  // MAV_CMD_NAV_WAYPOINT
            m.is_current = first;
            m.autocontinue = true;
            m.x_lat = wp.pos.lat * kRad2Deg;
            m.y_long = wp.pos.lon * kRad2Deg;
            m.z_alt = wp.pos.alt;
            req->waypoints.push_back(m);
            first = false;
        }
        if (!push_->wait_for_service(std::chrono::milliseconds(200))) {
            RCLCPP_ERROR(node_.get_logger(), "mavros /mavros/mission/push unavailable");
            return;
        }
        (void)push_->async_send_request(req);
        RCLCPP_WARN(node_.get_logger(), "[mavros] WaypointPush: %zu items", req->waypoints.size());
    }

private:
    rclcpp::Node& node_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_;
    rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr push_;
    rclcpp::Client<mavros_msgs::srv::WaypointClear>::SharedPtr clear_;
};
#endif  // HAVE_MAVROS

// --------------------------------------------------------------------------- //
// The failsafe node
// --------------------------------------------------------------------------- //
class ReturnHomeNode : public rclcpp::Node {
public:
    ReturnHomeNode() : rclcpp::Node("rth_node") {
        api::MissionConfig cfg;
        cfg.route.min_record_distance =
            declare_parameter("min_record_distance", cfg.route.min_record_distance);
        cfg.route.arrival_radius = declare_parameter("arrival_radius", cfg.route.arrival_radius);
        cfg.link.timeout = declare_parameter("link_timeout", cfg.link.timeout);
        cfg.link.resume_on_recovery =
            declare_parameter("resume_on_recovery", cfg.link.resume_on_recovery);
        cfg.simplify = declare_parameter("simplify", cfg.simplify);
        cfg.simplify_epsilon = declare_parameter("simplify_epsilon", cfg.simplify_epsilon);
        const double tick_rate = declare_parameter("tick_rate", 5.0);
        const bool use_mavros = declare_parameter("use_mavros", false);
        frame_id_ = declare_parameter("frame_id", std::string{"map"});

        link_ = makeLink(use_mavros);
        controller_.emplace(*link_, cfg);

        const auto qos = rclcpp::SensorDataQoS();
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            declare_parameter("position_topic", std::string{"/mavros/global_position/global"}), qos,
            [this](sensor_msgs::msg::NavSatFix::SharedPtr m) { onPosition(*m); });
        hb_sub_ = create_subscription<std_msgs::msg::Empty>(
            declare_parameter("heartbeat_topic", std::string{"/operator/heartbeat"}), 10,
            [this](std_msgs::msg::Empty::SharedPtr) { controller_->onHeartbeat(now_s()); });

        state_pub_ = create_publisher<std_msgs::msg::String>("/rth/mission_state", 10);
        recorded_pub_ = create_publisher<nav_msgs::msg::Path>("/rth/recorded_path", 10);

        force_srv_ = create_service<std_srvs::srv::Trigger>(
            "/rth/force", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                controller_->forceReturn(now_s());
                res->success = true;
                res->message = "return triggered";
            });

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / std::max(1.0, tick_rate)), [this] { onTick(); });
        RCLCPP_INFO(get_logger(),
                    "rth_node ready: recording trail, failsafe timeout=%.1fs, backend=%s.",
                    cfg.link.timeout, backend_name_.c_str());
    }

private:
    [[nodiscard]] std::unique_ptr<api::IAutopilotLink> makeLink(bool use_mavros) {
        if (use_mavros) {
#ifdef HAVE_MAVROS
            backend_name_ = "mavros";
            return std::make_unique<MavrosLink>(*this);
#else
            RCLCPP_WARN(get_logger(),
                        "use_mavros=true but built without mavros_msgs; using ROS bridge.");
#endif
        }
        backend_name_ = "ros_bridge";
        return std::make_unique<RosBridgeLink>(*this, frame_id_);
    }

    [[nodiscard]] double now_s() const { return now().seconds(); }

    void onPosition(const sensor_msgs::msg::NavSatFix& m) {
        if (std::isnan(m.latitude) || std::isnan(m.longitude)) return;
        const api::GeoPoint p{m.latitude * kDeg2Rad, m.longitude * kDeg2Rad, m.altitude};
        const auto before = controller_->mode();
        controller_->onPosition(p, now_s());
        logTransition(before);
    }

    void onTick() {
        const auto before = controller_->mode();
        controller_->tick(now_s());
        logTransition(before);
        publishState();
        publishRecordedPath();
    }

    void logTransition(api::MissionMode before) {
        const auto after = controller_->mode();
        if (after != before) {
            RCLCPP_INFO(get_logger(), "mission mode: %s -> %s", missionName(before).c_str(),
                        missionName(after).c_str());
        }
    }

    void publishState() {
        std_msgs::msg::String msg;
        msg.data = missionName(controller_->mode());
        state_pub_->publish(msg);
    }

    /// Publish the recorded breadcrumb trail as an ENU Path (origin = home) for
    /// RViz and downstream plotting.
    void publishRecordedPath() {
        const std::vector<api::GeoPoint> pts = controller_->routePoints();
        if (pts.size() < 2U) return;
        const api::GeoPoint home = pts.front();
        nav_msgs::msg::Path msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        for (const api::GeoPoint& p : pts) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.pose.position.x = (p.lon - home.lon) * std::cos(home.lat) * kEarthR;  // East
            ps.pose.position.y = (p.lat - home.lat) * kEarthR;                        // North
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        recorded_pub_->publish(msg);
    }

    std::unique_ptr<api::IAutopilotLink> link_;
    std::optional<api::MissionController> controller_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hb_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr recorded_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr force_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string frame_id_{"map"};
    std::string backend_name_{"ros_bridge"};
};

}  // namespace ugv::rth

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ugv::rth::ReturnHomeNode>());
    rclcpp::shutdown();
    return 0;
}
