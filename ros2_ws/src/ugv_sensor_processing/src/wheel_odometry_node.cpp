// ROS 2 sensor preprocessing: derive a body twist from wheel joint states using
// the ugv_nav_core differential-drive model (v = r/2(ωR+ωL), ωz = r/L(ωR−ωL)).
// Publishes an alternative wheel-odometry stream independent of the Gazebo
// DiffDrive plugin, illustrating core reuse inside ROS.
#include <ugv_nav_core/ugv_nav_core.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <string>

namespace ugv::sensor_processing {

namespace api = ugv::nav::api;

class WheelOdometryNode : public rclcpp::Node {
public:
    WheelOdometryNode() : rclcpp::Node("wheel_odometry_node") {
        radius_ = declare_parameter("wheel_radius", 0.10);
        track_ = declare_parameter("wheel_track", 0.36);
        left_joint_ = declare_parameter("left_joint", std::string{"left_wheel_joint"});
        right_joint_ = declare_parameter("right_joint", std::string{"right_wheel_joint"});

        sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::JointState::SharedPtr m) { onJoints(*m); });
        pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/wheel/twist", 10);
    }

private:
    void onJoints(const sensor_msgs::msg::JointState& m) {
        double wl = 0.0, wr = 0.0;
        bool have_l = false, have_r = false;
        for (std::size_t i = 0; i < m.name.size() && i < m.velocity.size(); ++i) {
            if (m.name[i] == left_joint_) { wl = m.velocity[i]; have_l = true; }
            else if (m.name[i] == right_joint_) { wr = m.velocity[i]; have_r = true; }
        }
        if (!have_l || !have_r) return;

        const auto twist = api::wheelOdometry(wl, wr, radius_, track_);
        geometry_msgs::msg::TwistStamped out;
        out.header = m.header;
        out.twist.linear.x = twist.v;
        out.twist.angular.z = twist.omega_z;
        pub_->publish(out);
    }

    double radius_{0.10};
    double track_{0.36};
    std::string left_joint_, right_joint_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
};

}  // namespace ugv::sensor_processing

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ugv::sensor_processing::WheelOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
