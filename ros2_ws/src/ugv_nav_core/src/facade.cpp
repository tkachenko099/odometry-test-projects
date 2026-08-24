// Facade implementation: bridges the ABI-stable `ugv::nav::api` surface to the
// C++23 `ugv.nav.core` module.
#include "ugv_nav_core/ugv_nav_core.hpp"

#include <vector>

import ugv.nav.core;

namespace ugv::nav::api {
namespace {

[[nodiscard]] EskfConfig toConfig(const EskfParams& p) {
    EskfConfig c;
    c.accel_noise = p.accel_noise;
    c.gyro_noise = p.gyro_noise;
    c.accel_bias_walk = p.accel_bias_walk;
    c.gyro_bias_walk = p.gyro_bias_walk;
    c.init_pos_std = p.init_pos_std;
    c.init_vel_std = p.init_vel_std;
    c.init_att_std = p.init_att_std_deg * math::kDeg2Rad;
    c.init_ba_std = p.init_ba_std;
    c.init_bg_std = p.init_bg_std;
    c.gravity = p.gravity;
    return c;
}

[[nodiscard]] NavState fromCore(const ugv::nav::NavState& s) {
    return {s.p, s.v, s.q, s.accel_bias, s.gyro_bias};
}

}  // namespace

struct Eskf::Impl {
    explicit Impl(const EskfParams& p) : filter(toConfig(p)) {}
    ErrorStateKf filter;
};

Eskf::Eskf(const EskfParams& params) : impl_(std::make_unique<Impl>(params)) {}
Eskf::~Eskf() = default;
Eskf::Eskf(Eskf&&) noexcept = default;
Eskf& Eskf::operator=(Eskf&&) noexcept = default;

void Eskf::reset(const NavState& x0) {
    ugv::nav::NavState s;
    s.p = x0.p;
    s.v = x0.v;
    s.q = x0.q;
    s.accel_bias = x0.accel_bias;
    s.gyro_bias = x0.gyro_bias;
    impl_->filter.reset(s);
}

void Eskf::predict(const Vec3& accel, const Vec3& gyro, Scalar t, Scalar dt) {
    impl_->filter.predict(ImuSample{.t = t, .accel = accel, .gyro = gyro}, dt);
}

void Eskf::updateGnssNed(const Vec3& p_ned, const Vec3& std_ned) {
    impl_->filter.updateGnssNed(p_ned, std_ned);
}

void Eskf::updateWheelSpeed(Scalar v_forward, Scalar std_v) {
    impl_->filter.updateWheelSpeed(v_forward, std_v);
}

NavState Eskf::state() const { return fromCore(impl_->filter.state()); }

Eigen::Matrix<Scalar, 15, 15> Eskf::covariance() const { return impl_->filter.covariance(); }

Vec3 llh2ned(const Geodetic& g, const Geodetic& ref) {
    return math::llh2ned(ugv::nav::Geodetic{g.lat, g.lon, g.alt},
                         ugv::nav::Geodetic{ref.lat, ref.lon, ref.alt});
}

Geodetic ned2llh(const Vec3& ned, const Geodetic& ref) {
    const auto g = math::ned2llh(ned, ugv::nav::Geodetic{ref.lat, ref.lon, ref.alt});
    return {g.lat, g.lon, g.alt};
}

Vec3 quat2euler(const Quat& q) { return math::quat2euler(q); }

BodyTwist wheelOdometry(Scalar omega_left, Scalar omega_right, Scalar radius, Scalar track) {
    const auto t = sensors::wheelOdometry(
        WheelSample{.t = 0.0, .omega_left = omega_left, .omega_right = omega_right}, radius,
        track);
    return {t.v, t.omega_z};
}

MetricReport evaluate(std::span<const PoseError> samples) {
    std::vector<ugv::nav::metrics::PoseError> core;
    core.reserve(samples.size());
    for (const auto& e : samples)
        core.push_back({e.t, e.p_est, e.p_gt, e.yaw_est, e.yaw_gt, e.v_est, e.v_gt});
    const auto r = ugv::nav::metrics::evaluate(core);
    return {r.position_rmse, r.ate,       r.rpe,
            r.velocity_rmse, r.yaw_rmse,  r.max_error,
            r.travelled_distance, r.relative_drift_pct, r.duration, r.samples};
}

}  // namespace ugv::nav::api
