// C++23 module partition: minimal, dependency-free geodesy for backtracking.
//
// Deliberately Eigen-free: the failsafe core stays self-contained and trivially
// portable to a resource-constrained onboard computer. All functions are pure,
// branch-bounded and noexcept.
module;

#include <cmath>

export module ugv.rth:geo;

import :types;

export namespace ugv::rth::geo {

inline constexpr Scalar kEarthRadius = 6'371'000.0;  ///< Mean Earth radius [m].

/// Great-circle distance between two geodetic points [m] (haversine).
[[nodiscard]] inline Scalar haversine(const GeoPoint& a, const GeoPoint& b) noexcept {
    const Scalar dlat = b.lat - a.lat;
    const Scalar dlon = b.lon - a.lon;
    const Scalar slat = std::sin(dlat * 0.5);
    const Scalar slon = std::sin(dlon * 0.5);
    const Scalar h = (slat * slat) + (std::cos(a.lat) * std::cos(b.lat) * slon * slon);
    return 2.0 * kEarthRadius * std::asin(std::sqrt(std::min(1.0, h)));
}

/// Initial bearing from `a` to `b` [rad], measured clockwise from north.
[[nodiscard]] inline Scalar initialBearing(const GeoPoint& a, const GeoPoint& b) noexcept {
    const Scalar dlon = b.lon - a.lon;
    const Scalar y = std::sin(dlon) * std::cos(b.lat);
    const Scalar x = (std::cos(a.lat) * std::sin(b.lat)) -
                     (std::sin(a.lat) * std::cos(b.lat) * std::cos(dlon));
    return std::atan2(y, x);
}

/// Local tangent-plane (equirectangular) coordinates of `p` relative to
/// `origin` [m]. Accurate for the short segments used in trail simplification.
struct PlanarPoint {
    Scalar x{0.0};  ///< East [m]
    Scalar y{0.0};  ///< North [m]
};

[[nodiscard]] inline PlanarPoint project(const GeoPoint& p, const GeoPoint& origin) noexcept {
    return {(p.lon - origin.lon) * std::cos(origin.lat) * kEarthRadius,
            (p.lat - origin.lat) * kEarthRadius};
}

/// Perpendicular distance from point `p` to the segment `[a, b]` [m], computed
/// in the local tangent plane anchored at `a`.
[[nodiscard]] inline Scalar perpendicularDistance(const GeoPoint& p, const GeoPoint& a,
                                                   const GeoPoint& b) noexcept {
    const PlanarPoint pp = project(p, a);
    const PlanarPoint bb = project(b, a);
    const Scalar len2 = (bb.x * bb.x) + (bb.y * bb.y);
    if (len2 < 1e-12) {
        return std::sqrt((pp.x * pp.x) + (pp.y * pp.y));  // degenerate segment
    }
    // Signed area of the parallelogram / base length = perpendicular height.
    const Scalar cross = std::abs((bb.x * pp.y) - (bb.y * pp.x));
    return cross / std::sqrt(len2);
}

}  // namespace ugv::rth::geo
