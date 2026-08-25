// C++23 module partition: the return-home failsafe state machine.
module;

#include <cstddef>
#include <span>

export module ugv.rth:mission;

import :types;
import :geo;
import :route;
import :simplify;
import :link;

export namespace ugv::rth {

/// Abstraction of the autopilot datalink (implemented by a MAVLink/mavros
/// backend, a ROS bridge, or a simulator). Kept non-owning and side-effect
/// only; the controller never blocks on it.
class IAutopilotLink {
public:
    IAutopilotLink() = default;
    IAutopilotLink(const IAutopilotLink&) = default;
    IAutopilotLink(IAutopilotLink&&) = default;
    IAutopilotLink& operator=(const IAutopilotLink&) = default;
    IAutopilotLink& operator=(IAutopilotLink&&) = default;
    virtual ~IAutopilotLink() = default;

    /// Command the autopilot flight/drive mode (MAVLink SET_MODE).
    virtual void setMode(AutopilotMode mode) = 0;

    /// Upload the backtracking mission (MAVLink MISSION_COUNT + MISSION_ITEM_INT).
    virtual void uploadReturnPath(std::span<const Waypoint> path) = 0;
};

/// Onboard failsafe controller: records the operator-driven trail and, on link
/// loss, autonomously commands the autopilot to backtrack home along it.
///
/// The class owns no threads and performs no allocation on its control path;
/// callers drive it with `onHeartbeat`, `onPosition` and periodic `tick`.
class MissionController {
public:
    MissionController(IAutopilotLink& link, MissionConfig cfg) noexcept
        : link_(link), cfg_(cfg), recorder_(cfg.route), monitor_(cfg.link) {}

    // Non-copyable (holds a link reference); movable is unnecessary here.
    MissionController(const MissionController&) = delete;
    MissionController& operator=(const MissionController&) = delete;

    /// Operator heartbeat at time `t`.
    void onHeartbeat(Stamp t) noexcept {
        monitor_.heartbeat(t);
        if (mode_ == MissionMode::kReturning && cfg_.link.resume_on_recovery) {
            abortReturn();
        }
    }

    /// New vehicle position (from GNSS / autopilot GLOBAL_POSITION_INT).
    void onPosition(const GeoPoint& p, Stamp t) {
        current_ = p;
        have_current_ = true;
        if (mode_ == MissionMode::kIdle) {
            mode_ = MissionMode::kRecording;
        }
        if (mode_ == MissionMode::kRecording) {
            (void)recorder_.record(p);
        }
        evaluate(t);
    }

    /// Periodic evaluation (call at a fixed rate, e.g. 5-10 Hz).
    void tick(Stamp now) { evaluate(now); }

    /// Manually trigger the backtrack (e.g. operator "return" button).
    void forceReturn(Stamp now) {
        if (mode_ == MissionMode::kRecording) {
            triggerReturn(now);
        }
    }

    // --- Observers -----------------------------------------------------------
    [[nodiscard]] MissionMode mode() const noexcept { return mode_; }
    [[nodiscard]] LinkState linkState(Stamp now) const noexcept { return monitor_.state(now); }
    [[nodiscard]] std::span<const Waypoint> returnPath() const noexcept { return return_path_.view(); }
    [[nodiscard]] const RouteRecorder& route() const noexcept { return recorder_; }
    [[nodiscard]] std::size_t routeSize() const noexcept { return recorder_.size(); }
    [[nodiscard]] bool hasHome() const noexcept { return !recorder_.empty(); }

    /// Straight-line distance from the current position to home [m].
    [[nodiscard]] Scalar distanceToHome() const noexcept {
        if (!have_current_ || recorder_.empty()) {
            return 0.0;
        }
        return geo::haversine(current_, recorder_.home());
    }

    void reset() noexcept {
        recorder_.clear();
        monitor_.reset();
        return_path_.clear();
        mode_ = MissionMode::kIdle;
        have_current_ = false;
    }

private:
    void evaluate(Stamp now) {
        switch (mode_) {
            case MissionMode::kRecording:
                if (monitor_.lost(now) && recorder_.size() >= 2U) {
                    triggerReturn(now);
                }
                break;
            case MissionMode::kReturning:
                if (have_current_ && !recorder_.empty() &&
                    distanceToHome() <= cfg_.route.arrival_radius) {
                    mode_ = MissionMode::kHome;
                    link_.setMode(AutopilotMode::kHold);
                }
                break;
            case MissionMode::kIdle:
            case MissionMode::kHome:
            default:
                break;
        }
    }

    void triggerReturn(Stamp /*now*/) {
        buildReturnPath(recorder_.points(), cfg_, return_path_);
        link_.setMode(AutopilotMode::kGuided);
        link_.uploadReturnPath(return_path_.view());
        link_.setMode(AutopilotMode::kAuto);
        mode_ = MissionMode::kReturning;
    }

    void abortReturn() {
        return_path_.clear();
        mode_ = MissionMode::kRecording;
        link_.setMode(AutopilotMode::kManual);
    }

    IAutopilotLink& link_;
    MissionConfig cfg_{};
    RouteRecorder recorder_{};
    LinkMonitor monitor_{};
    StaticVector<Waypoint, kMaxWaypoints> return_path_{};
    GeoPoint current_{};
    bool have_current_{false};
    MissionMode mode_{MissionMode::kIdle};
};

}  // namespace ugv::rth
