// C++23 module partition: attitude algebra (SO3) and WGS-84 geodesy.
module;

#include <Eigen/Dense>
#include <cmath>
#include <numbers>

export module ugv.nav.core:math;

import :types;

export namespace ugv::nav::math {

inline constexpr Scalar kPi = std::numbers::pi_v<Scalar>;
inline constexpr Scalar kDeg2Rad = kPi / 180.0;
inline constexpr Scalar kRad2Deg = 180.0 / kPi;

// --- WGS-84 ellipsoid ------------------------------------------------------
inline constexpr Scalar kWgs84A = 6'378'137.0;             ///< Semi-major axis [m]
inline constexpr Scalar kWgs84F = 1.0 / 298.257'223'563;   ///< Flattening
inline constexpr Scalar kWgs84E2 = kWgs84F * (2.0 - kWgs84F);  ///< First eccentricity^2
inline constexpr Scalar kGravity = 9.806'65;               ///< Nominal gravity [m/s^2]

/// Skew-symmetric (cross-product) matrix of a 3-vector.
[[nodiscard]] inline Mat3 skew(const Vec3& v) noexcept {
    Mat3 m;
    m << 0.0, -v.z(), v.y(),
         v.z(), 0.0, -v.x(),
        -v.y(), v.x(), 0.0;
    return m;
}

/// Exponential map so(3) -> SO(3) returning a unit quaternion.
[[nodiscard]] inline Quat expq(const Vec3& phi) noexcept {
    const Scalar angle = phi.norm();
    if (angle < 1e-12) {
        Quat q(1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z());
        q.normalize();
        return q;
    }
    const Vec3 axis = phi / angle;
    const Scalar half = 0.5 * angle;
    const Scalar s = std::sin(half);
    return Quat(std::cos(half), axis.x() * s, axis.y() * s, axis.z() * s);
}

/// Logarithm map SO(3) -> so(3): rotation vector of a quaternion.
[[nodiscard]] inline Vec3 logq(const Quat& q_in) noexcept {
    Quat q = q_in.normalized();
    if (q.w() < 0.0) q.coeffs() *= -1.0;  // shortest-arc branch
    const Vec3 vec = q.vec();
    const Scalar n = vec.norm();
    if (n < 1e-12) return 2.0 * vec;
    return 2.0 * std::atan2(n, q.w()) * (vec / n);
}

/// Right-multiplicative attitude update q ⊗ Exp(phi).
[[nodiscard]] inline Quat boxplus(const Quat& q, const Vec3& phi) noexcept {
    return (q * expq(phi)).normalized();
}

/// Rotation matrix -> yaw/pitch/roll (ZYX) Euler angles [rad].
[[nodiscard]] inline Vec3 rot2euler(const Mat3& R) noexcept {
    Vec3 e;                                    // (roll, pitch, yaw)
    e.y() = std::asin(-std::clamp(R(2, 0), -1.0, 1.0));
    if (std::abs(R(2, 0)) < 0.999'999) {
        e.x() = std::atan2(R(2, 1), R(2, 2));
        e.z() = std::atan2(R(1, 0), R(0, 0));
    } else {                                   // gimbal-lock fallback
        e.x() = std::atan2(-R(1, 2), R(1, 1));
        e.z() = 0.0;
    }
    return e;
}

[[nodiscard]] inline Vec3 quat2euler(const Quat& q) noexcept {
    return rot2euler(q.toRotationMatrix());
}

/// Meridian/normal radii of curvature at a given latitude.
[[nodiscard]] inline std::pair<Scalar, Scalar> radii(Scalar lat) noexcept {
    const Scalar s = std::sin(lat);
    const Scalar w = 1.0 - kWgs84E2 * s * s;
    const Scalar rn = kWgs84A / std::sqrt(w);             // prime vertical
    const Scalar rm = kWgs84A * (1.0 - kWgs84E2) / (w * std::sqrt(w));  // meridian
    return {rm, rn};
}

/// Geodetic (rad,rad,m) -> ECEF (m).
[[nodiscard]] inline Vec3 llh2ecef(const Geodetic& g) noexcept {
    const auto [rm, rn] = radii(g.lat);
    (void)rm;
    const Scalar cl = std::cos(g.lat), sl = std::sin(g.lat);
    const Scalar co = std::cos(g.lon), so = std::sin(g.lon);
    return {(rn + g.alt) * cl * co,
            (rn + g.alt) * cl * so,
            (rn * (1.0 - kWgs84E2) + g.alt) * sl};
}

/// Rotation from ECEF to local NED at the reference geodetic point.
[[nodiscard]] inline Mat3 ecef2ned_rotation(const Geodetic& ref) noexcept {
    const Scalar cl = std::cos(ref.lat), sl = std::sin(ref.lat);
    const Scalar co = std::cos(ref.lon), so = std::sin(ref.lon);
    Mat3 r;
    r << -sl * co, -sl * so, cl,
         -so, co, 0.0,
         -cl * co, -cl * so, -sl;
    return r;
}

/// Geodetic -> local NED (m) relative to a reference origin.
[[nodiscard]] inline Vec3 llh2ned(const Geodetic& g, const Geodetic& ref) noexcept {
    return ecef2ned_rotation(ref) * (llh2ecef(g) - llh2ecef(ref));
}

/// Local NED (m) -> geodetic, inverse of llh2ned (iteration-free, linearised
/// via curvature radii — accurate for the small offsets used in this stand).
[[nodiscard]] inline Geodetic ned2llh(const Vec3& ned, const Geodetic& ref) noexcept {
    const auto [rm, rn] = radii(ref.lat);
    return {ref.lat + ned.x() / (rm + ref.alt),
            ref.lon + ned.y() / ((rn + ref.alt) * std::cos(ref.lat)),
            ref.alt - ned.z()};
}

}  // namespace ugv::nav::math
