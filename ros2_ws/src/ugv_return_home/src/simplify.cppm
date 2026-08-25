// C++23 module partition: Douglas-Peucker trail simplification and construction
// of the reversed return mission handed to the autopilot.
module;

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

export module ugv.rth:simplify;

import :types;
import :geo;
import :route;

export namespace ugv::rth {

/// Work-stack capacity for the iterative Douglas-Peucker: the number of kept
/// vertices never exceeds the mission cap, so pending segments are bounded too.
inline constexpr std::size_t kDpStackCapacity = kMaxWaypoints + 2U;

/// Iterative (recursion-free, MISRA-friendly) Douglas-Peucker simplification.
/// Writes the retained vertices in input order into `out`; returns their count.
[[nodiscard]] inline std::size_t douglasPeucker(std::span<const GeoPoint> in, Scalar epsilon,
                                                StaticVector<GeoPoint, kMaxWaypoints>& out) {
    out.clear();
    const std::size_t n = in.size();
    if (n <= 2U) {
        for (std::size_t i = 0U; i < n; ++i) {
            (void)out.push_back(in[i]);
        }
        return out.size();
    }

    std::array<bool, kMaxRoutePoints> keep{};  // value-initialised to false
    keep[0] = true;
    keep[n - 1U] = true;

    StaticVector<std::pair<std::size_t, std::size_t>, kDpStackCapacity> stack{};
    (void)stack.push_back({0U, n - 1U});

    while (!stack.empty()) {
        const std::pair<std::size_t, std::size_t> seg = stack.back();
        stack.pop_back();
        const std::size_t s = seg.first;
        const std::size_t e = seg.second;
        if (e <= s + 1U) {
            continue;  // no interior vertices
        }

        Scalar max_d = 0.0;
        std::size_t max_i = s;
        for (std::size_t i = s + 1U; i < e; ++i) {
            const Scalar d = geo::perpendicularDistance(in[i], in[s], in[e]);
            if (d > max_d) {
                max_d = d;
                max_i = i;
            }
        }

        if (max_d > epsilon) {
            keep[max_i] = true;
            // If the stack is saturated we stop subdividing (safe coarsening).
            (void)stack.push_back({s, max_i});
            (void)stack.push_back({max_i, e});
        }
    }

    for (std::size_t i = 0U; i < n; ++i) {
        if (keep[i]) {
            (void)out.push_back(in[i]);
        }
    }
    return out.size();
}

/// Build the return mission: reverse the recorded trail so the vehicle heads
/// from its current position back to the launch point, optionally simplifying
/// it first, and assign monotonically increasing MAVLink sequence numbers.
inline void buildReturnPath(std::span<const GeoPoint> trail, const MissionConfig& cfg,
                            StaticVector<Waypoint, kMaxWaypoints>& out) {
    out.clear();
    if (trail.empty()) {
        return;
    }

    StaticVector<GeoPoint, kMaxWaypoints> kept{};
    if (cfg.simplify) {
        // Adaptively raise epsilon until the mission fits the autopilot cap.
        Scalar eps = cfg.simplify_epsilon;
        for (std::size_t attempt = 0U; attempt < 16U; ++attempt) {
            const std::size_t k = douglasPeucker(trail, eps, kept);
            if (k <= kMaxWaypoints) {
                break;
            }
            eps *= 1.75;
        }
    } else {
        for (std::size_t i = 0U; i < trail.size() && !kept.full(); ++i) {
            (void)kept.push_back(trail[i]);
        }
    }

    // Emit in reverse (current -> ... -> home) with fresh sequence numbers.
    std::uint16_t seq = 0U;
    for (std::size_t i = kept.size(); i-- > 0U;) {
        (void)out.push_back(Waypoint{kept[i], seq});
        ++seq;
    }
}

}  // namespace ugv::rth
