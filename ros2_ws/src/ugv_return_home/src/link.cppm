// C++23 module partition: operator-datalink heartbeat supervision.
module;

#include <limits>
#include <optional>

export module ugv.rth:link;

import :types;

export namespace ugv::rth {

/// Watches operator heartbeats (MAVLink HEARTBEAT from the GCS) and reports the
/// link as lost once none have arrived within `timeout` seconds.
class LinkMonitor {
public:
    LinkMonitor() = default;
    explicit LinkMonitor(LinkConfig cfg) noexcept : cfg_(cfg) {}

    /// Register a heartbeat received at time `t`.
    void heartbeat(Stamp t) noexcept { last_ = t; }

    [[nodiscard]] LinkState state(Stamp now) const noexcept {
        if (!last_.has_value()) {
            return LinkState::kUninitialised;
        }
        return (now - *last_) > cfg_.timeout ? LinkState::kLost : LinkState::kConnected;
    }

    [[nodiscard]] bool connected(Stamp now) const noexcept {
        return state(now) == LinkState::kConnected;
    }

    [[nodiscard]] bool lost(Stamp now) const noexcept {
        return state(now) == LinkState::kLost;
    }

    /// Seconds since the last heartbeat (∞ sentinel if none seen yet).
    [[nodiscard]] Scalar sinceLastBeat(Stamp now) const noexcept {
        return last_.has_value() ? (now - *last_) : std::numeric_limits<Scalar>::infinity();
    }

    [[nodiscard]] const LinkConfig& config() const noexcept { return cfg_; }

    void reset() noexcept { last_.reset(); }

private:
    LinkConfig cfg_{};
    std::optional<Stamp> last_{};
};

}  // namespace ugv::rth
