// C++23 module partition: analytic sensor models (wheel odometry, GNSS conv).
module;

#include <Eigen/Dense>

export module ugv.nav.core:sensors;

import :types;
import :math;

export namespace ugv::nav::sensors {

/// Differential-drive body twist derived from wheel angular rates.
struct BodyTwist {
    Scalar v{0.0};        ///< Forward linear velocity [m/s]
    Scalar omega_z{0.0};  ///< Yaw rate [rad/s]
};

/// Differential-drive forward kinematics:
///   v      = r/2 (ω_R + ω_L)
///   ω_z    = r/L (ω_R − ω_L)
/// with wheel radius `r` [m] and track width `L` [m].
[[nodiscard]] inline BodyTwist wheelOdometry(const WheelSample& w, Scalar radius,
                                             Scalar track) noexcept {
    const Scalar v = 0.5 * radius * (w.omega_right + w.omega_left);
    const Scalar wz = (track > 0.0) ? radius / track * (w.omega_right - w.omega_left) : 0.0;
    return {v, wz};
}

/// Convert a GNSS geodetic fix to the local NED frame about `ref`.
[[nodiscard]] inline Vec3 gnssToNed(const GnssSample& fix, const Geodetic& ref) noexcept {
    return math::llh2ned(fix.llh, ref);
}

}  // namespace ugv::nav::sensors
