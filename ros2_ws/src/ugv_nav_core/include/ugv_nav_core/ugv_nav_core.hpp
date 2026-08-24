// Public C++ facade for the `ugv.nav.core` module.
//
// C++20 named modules cannot yet be reliably re-exported across independent
// colcon/ament packages, so ROS 2 nodes consume the navigation core through
// this stable header + linked static library. The implementation
// (`facade.cpp`) `import`s the module internally, keeping the algorithmic core
// module-based while presenting an ABI-stable, include-only surface here.
#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace ugv::nav::api {

using Scalar = double;
using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

struct Geodetic {
    Scalar lat{0.0};  ///< [rad]
    Scalar lon{0.0};  ///< [rad]
    Scalar alt{0.0};  ///< [m]
};

struct NavState {
    Vec3 p{Vec3::Zero()};
    Vec3 v{Vec3::Zero()};
    Quat q{Quat::Identity()};
    Vec3 accel_bias{Vec3::Zero()};
    Vec3 gyro_bias{Vec3::Zero()};
};

struct EskfParams {
    Scalar accel_noise{1.0e-2};
    Scalar gyro_noise{1.0e-3};
    Scalar accel_bias_walk{1.0e-4};
    Scalar gyro_bias_walk{1.0e-5};
    Scalar init_pos_std{1.0};
    Scalar init_vel_std{0.5};
    Scalar init_att_std_deg{5.0};
    Scalar init_ba_std{0.1};
    Scalar init_bg_std{0.01};
    Vec3 gravity{0.0, 0.0, 9.80665};
};

/// Thin PIMPL wrapper around `ugv::nav::ErrorStateKf` (defined in the module).
class Eskf {
public:
    explicit Eskf(const EskfParams& params = {});
    ~Eskf();
    Eskf(Eskf&&) noexcept;
    Eskf& operator=(Eskf&&) noexcept;
    Eskf(const Eskf&) = delete;
    Eskf& operator=(const Eskf&) = delete;

    void reset(const NavState& x0);
    void predict(const Vec3& accel, const Vec3& gyro, Scalar t, Scalar dt);
    void updateGnssNed(const Vec3& p_ned, const Vec3& std_ned);
    void updateWheelSpeed(Scalar v_forward, Scalar std_v);

    [[nodiscard]] NavState state() const;
    [[nodiscard]] Eigen::Matrix<Scalar, 15, 15> covariance() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// --- Free-function geodesy & sensor helpers -------------------------------
[[nodiscard]] Vec3 llh2ned(const Geodetic& g, const Geodetic& ref);
[[nodiscard]] Geodetic ned2llh(const Vec3& ned, const Geodetic& ref);
[[nodiscard]] Vec3 quat2euler(const Quat& q);  ///< returns (roll, pitch, yaw) [rad]

struct BodyTwist {
    Scalar v{0.0};
    Scalar omega_z{0.0};
};
[[nodiscard]] BodyTwist wheelOdometry(Scalar omega_left, Scalar omega_right, Scalar radius,
                                      Scalar track);

// --- Metrics --------------------------------------------------------------
struct PoseError {
    Scalar t{0.0};
    Vec3 p_est{Vec3::Zero()};
    Vec3 p_gt{Vec3::Zero()};
    Scalar yaw_est{0.0};
    Scalar yaw_gt{0.0};
    Vec3 v_est{Vec3::Zero()};
    Vec3 v_gt{Vec3::Zero()};
};

struct MetricReport {
    Scalar position_rmse{0.0};
    Scalar ate{0.0};
    Scalar rpe{0.0};
    Scalar velocity_rmse{0.0};
    Scalar yaw_rmse{0.0};
    Scalar max_error{0.0};
    Scalar travelled_distance{0.0};
    Scalar relative_drift_pct{0.0};
    Scalar duration{0.0};
    std::size_t samples{0};
};

[[nodiscard]] MetricReport evaluate(std::span<const PoseError> samples);

}  // namespace ugv::nav::api
