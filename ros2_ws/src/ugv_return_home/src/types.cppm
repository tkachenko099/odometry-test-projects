// C++23 module partition: fundamental types for the return-home failsafe.
//
// MISRA-friendly conventions applied throughout this module:
//   * strong `enum class` types with a fixed underlying type,
//   * fixed-width integers, no implicit narrowing,
//   * value semantics only (no owning raw pointers),
//   * bounded, statically-sized storage (see :route),
//   * `[[nodiscard]]` / `noexcept` on pure queries,
//   * no exceptions on the control path (status returned by value).
module;

#include <cstddef>
#include <cstdint>

export module ugv.rth:types;

export namespace ugv::rth {

using Scalar = double;   ///< SI floating type (metres, radians, seconds).
using Stamp = double;    ///< Monotonic timestamp [s].

/// Geodetic position (WGS-84). Angles in **radians**, altitude in metres.
struct GeoPoint {
    Scalar lat{0.0};  ///< Latitude  [rad]
    Scalar lon{0.0};  ///< Longitude [rad]
    Scalar alt{0.0};  ///< Altitude (AMSL/relative) [m]
};

/// A single mission item to hand to the autopilot (MAVLink MISSION_ITEM_INT).
struct Waypoint {
    GeoPoint pos{};        ///< Target geodetic position.
    std::uint16_t seq{0};  ///< Mission sequence index (0 = first to fly).
};

/// Commanded ArduPilot flight/drive mode (subset relevant to backtracking).
enum class AutopilotMode : std::uint8_t {
    kUnknown = 0,
    kManual,          ///< Operator in control.
    kHold,            ///< Stationary / loiter.
    kGuided,          ///< Accepts external position targets.
    kAuto,            ///< Executes the uploaded mission.
    kReturnToLaunch,  ///< Native RTL (straight-line home).
};

/// High-level onboard mission state (the failsafe state machine).
enum class MissionMode : std::uint8_t {
    kIdle = 0,     ///< Awaiting first fix + operator heartbeat.
    kRecording,    ///< Under operator control; logging the breadcrumb trail.
    kReturning,    ///< Link lost: backtracking home along the recorded trail.
    kHome,         ///< Arrived back at the launch point.
};

/// Operator datalink health as judged by the heartbeat monitor.
enum class LinkState : std::uint8_t {
    kUninitialised = 0,  ///< No heartbeat seen yet.
    kConnected,          ///< Recent heartbeat within timeout.
    kLost,               ///< Heartbeat overdue -> failsafe.
};

/// Breadcrumb-recording policy.
struct RouteConfig {
    Scalar min_record_distance{1.0};  ///< Decimation: skip fixes closer than this [m].
    Scalar arrival_radius{2.0};       ///< "Home reached" tolerance [m].
};

/// Operator-link supervision policy.
struct LinkConfig {
    Scalar timeout{3.0};             ///< Heartbeat age that trips the failsafe [s].
    bool resume_on_recovery{true};   ///< Abort the return if the link comes back.
};

/// Aggregate mission configuration.
struct MissionConfig {
    RouteConfig route{};
    LinkConfig link{};
    bool simplify{true};              ///< Douglas-Peucker the trail before upload.
    Scalar simplify_epsilon{0.75};    ///< DP tolerance [m].
};

/// Bounded storage limits (no dynamic allocation on the control path).
inline constexpr std::size_t kMaxRoutePoints = 8192U;  ///< Breadcrumb capacity.
inline constexpr std::size_t kMaxWaypoints = 512U;      ///< ArduPilot mission cap.

}  // namespace ugv::rth
