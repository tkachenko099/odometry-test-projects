// Public C++ facade for the `ugv.rth` return-home failsafe module.
//
// As with `ugv_nav_core`, C++20/23 named modules are not re-exported across
// independent colcon packages, so ROS 2 nodes and other consumers use this
// ABI-stable header + linked library. `facade.cpp` `import`s the module and
// bridges to it, keeping the algorithmic core fully module-based.
#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace ugv::rth::api {

using Scalar = double;
using Stamp = double;

/// Geodetic position (WGS-84): angles in **radians**, altitude in metres.
struct GeoPoint {
    Scalar lat{0.0};
    Scalar lon{0.0};
    Scalar alt{0.0};
};

/// Mission item handed to the autopilot (MAVLink MISSION_ITEM_INT).
struct Waypoint {
    GeoPoint pos{};
    std::uint16_t seq{0};
};

// Enum values mirror the module 1:1 (bridged by static_cast in facade.cpp).
enum class AutopilotMode : std::uint8_t {
    kUnknown = 0, kManual, kHold, kGuided, kAuto, kReturnToLaunch
};
enum class MissionMode : std::uint8_t { kIdle = 0, kRecording, kReturning, kHome };
enum class LinkState : std::uint8_t { kUninitialised = 0, kConnected, kLost };

struct RouteConfig {
    Scalar min_record_distance{1.0};
    Scalar arrival_radius{2.0};
};
struct LinkConfig {
    Scalar timeout{3.0};
    bool resume_on_recovery{true};
};
struct MissionConfig {
    RouteConfig route{};
    LinkConfig link{};
    bool simplify{true};
    Scalar simplify_epsilon{0.75};
};

/// Autopilot datalink abstraction. Implement this with a MAVLink/mavros
/// backend (real vehicle) or a simulator/ROS bridge (this stand).
class IAutopilotLink {
public:
    IAutopilotLink() = default;
    IAutopilotLink(const IAutopilotLink&) = default;
    IAutopilotLink(IAutopilotLink&&) = default;
    IAutopilotLink& operator=(const IAutopilotLink&) = default;
    IAutopilotLink& operator=(IAutopilotLink&&) = default;
    virtual ~IAutopilotLink() = default;

    virtual void setMode(AutopilotMode mode) = 0;
    virtual void uploadReturnPath(std::span<const Waypoint> path) = 0;
};

/// Great-circle distance between two geodetic points [m].
[[nodiscard]] Scalar distanceMeters(const GeoPoint& a, const GeoPoint& b);

/// PIMPL wrapper around the module `ugv::rth::MissionController`.
class MissionController {
public:
    MissionController(IAutopilotLink& link, const MissionConfig& cfg = {});
    ~MissionController();
    MissionController(MissionController&&) noexcept;
    MissionController& operator=(MissionController&&) noexcept;
    MissionController(const MissionController&) = delete;
    MissionController& operator=(const MissionController&) = delete;

    void onHeartbeat(Stamp t);
    void onPosition(const GeoPoint& p, Stamp t);
    void tick(Stamp now);
    void forceReturn(Stamp now);
    void reset();

    [[nodiscard]] MissionMode mode() const;
    [[nodiscard]] LinkState linkState(Stamp now) const;
    [[nodiscard]] bool hasHome() const;
    [[nodiscard]] Scalar distanceToHome() const;
    [[nodiscard]] std::size_t routeSize() const;
    [[nodiscard]] std::vector<Waypoint> returnPath() const;
    [[nodiscard]] std::vector<GeoPoint> routePoints() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace ugv::rth::api
