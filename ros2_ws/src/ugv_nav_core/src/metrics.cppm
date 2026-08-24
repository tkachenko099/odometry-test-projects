// C++23 module partition: navigation accuracy metrics (RMSE / ATE / RPE / drift).
module;

#include <Eigen/Dense>
#include <cmath>
#include <ranges>
#include <span>
#include <stdexcept>
#include <vector>

export module ugv.nav.core:metrics;

import :types;
import :math;

export namespace ugv::nav::metrics {

/// A time-stamped pose pair (estimate vs. ground truth) for evaluation.
struct PoseError {
    Stamp t{0.0};
    Vec3 p_est{Vec3::Zero()};
    Vec3 p_gt{Vec3::Zero()};
    Scalar yaw_est{0.0};
    Scalar yaw_gt{0.0};
    Vec3 v_est{Vec3::Zero()};
    Vec3 v_gt{Vec3::Zero()};
};

/// Aggregated metric bundle mirroring the dashboard cards (Section 17).
struct Report {
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

/// Wrap an angle difference to (−π, π].
[[nodiscard]] inline Scalar wrapPi(Scalar a) noexcept {
    while (a > math::kPi) a -= 2.0 * math::kPi;
    while (a <= -math::kPi) a += 2.0 * math::kPi;
    return a;
}

/// Root-mean-square of a range of scalar residuals.
template <std::ranges::input_range R>
    requires std::convertible_to<std::ranges::range_value_t<R>, Scalar>
[[nodiscard]] Scalar rmse(R&& residuals) {
    Scalar acc = 0.0;
    std::size_t n = 0;
    for (const Scalar r : residuals) { acc += r * r; ++n; }
    return n ? std::sqrt(acc / static_cast<Scalar>(n)) : 0.0;
}

/// 3-D position RMSE over aligned estimate/ground-truth samples.
[[nodiscard]] inline Scalar positionRmse(std::span<const PoseError> s) {
    return rmse(s | std::views::transform(
                        [](const PoseError& e) { return (e.p_est - e.p_gt).norm(); }));
}

/// Absolute Trajectory Error (RMS of positional discrepancy).
[[nodiscard]] inline Scalar ate(std::span<const PoseError> s) { return positionRmse(s); }

/// Velocity RMSE (3-D).
[[nodiscard]] inline Scalar velocityRmse(std::span<const PoseError> s) {
    return rmse(s | std::views::transform(
                        [](const PoseError& e) { return (e.v_est - e.v_gt).norm(); }));
}

/// Yaw RMSE with proper angular wrapping.
[[nodiscard]] inline Scalar yawRmse(std::span<const PoseError> s) {
    return rmse(s | std::views::transform(
                        [](const PoseError& e) { return wrapPi(e.yaw_est - e.yaw_gt); }));
}

/// Maximum instantaneous position error.
[[nodiscard]] inline Scalar maxError(std::span<const PoseError> s) {
    Scalar m = 0.0;
    for (const auto& e : s) m = std::max(m, (e.p_est - e.p_gt).norm());
    return m;
}

/// Relative Pose Error: RMS of frame-to-frame translational drift over a
/// fixed sample stride `delta`.
[[nodiscard]] inline Scalar rpe(std::span<const PoseError> s, std::size_t delta = 1) {
    if (s.size() <= delta) return 0.0;
    Scalar acc = 0.0;
    std::size_t n = 0;
    for (std::size_t i = 0; i + delta < s.size(); ++i) {
        const Vec3 d_est = s[i + delta].p_est - s[i].p_est;
        const Vec3 d_gt = s[i + delta].p_gt - s[i].p_gt;
        acc += (d_est - d_gt).squaredNorm();
        ++n;
    }
    return n ? std::sqrt(acc / static_cast<Scalar>(n)) : 0.0;
}

/// Ground-truth travelled path length.
[[nodiscard]] inline Scalar travelledDistance(std::span<const PoseError> s) {
    Scalar d = 0.0;
    for (std::size_t i = 1; i < s.size(); ++i) d += (s[i].p_gt - s[i - 1].p_gt).norm();
    return d;
}

/// Compute the full metric report in a single pass-friendly wrapper.
[[nodiscard]] inline Report evaluate(std::span<const PoseError> s) {
    Report r;
    r.samples = s.size();
    if (s.empty()) return r;
    r.position_rmse = positionRmse(s);
    r.ate = r.position_rmse;
    r.rpe = rpe(s);
    r.velocity_rmse = velocityRmse(s);
    r.yaw_rmse = yawRmse(s);
    r.max_error = maxError(s);
    r.travelled_distance = travelledDistance(s);
    r.duration = s.back().t - s.front().t;
    r.relative_drift_pct =
        r.travelled_distance > 1e-6 ? 100.0 * r.max_error / r.travelled_distance : 0.0;
    return r;
}

}  // namespace ugv::nav::metrics
