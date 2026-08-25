// Facade implementation: bridges the ABI-stable `ugv::rth::api` surface to the
// C++23 `ugv.rth` module.
#include "ugv_return_home/ugv_return_home.hpp"

#include <vector>

import ugv.rth;

namespace ugv::rth::api {
namespace {

// --- enum / struct conversions (module <-> api). Values are kept identical. ---
[[nodiscard]] ugv::rth::AutopilotMode toCore(AutopilotMode m) noexcept {
    return static_cast<ugv::rth::AutopilotMode>(static_cast<std::uint8_t>(m));
}
[[nodiscard]] AutopilotMode fromCore(ugv::rth::AutopilotMode m) noexcept {
    return static_cast<AutopilotMode>(static_cast<std::uint8_t>(m));
}
[[nodiscard]] MissionMode fromCore(ugv::rth::MissionMode m) noexcept {
    return static_cast<MissionMode>(static_cast<std::uint8_t>(m));
}
[[nodiscard]] LinkState fromCore(ugv::rth::LinkState s) noexcept {
    return static_cast<LinkState>(static_cast<std::uint8_t>(s));
}
[[nodiscard]] ugv::rth::GeoPoint toCore(const GeoPoint& p) noexcept { return {p.lat, p.lon, p.alt}; }
[[nodiscard]] GeoPoint fromCore(const ugv::rth::GeoPoint& p) noexcept { return {p.lat, p.lon, p.alt}; }

[[nodiscard]] ugv::rth::MissionConfig toCore(const MissionConfig& c) noexcept {
    ugv::rth::MissionConfig o;
    o.route.min_record_distance = c.route.min_record_distance;
    o.route.arrival_radius = c.route.arrival_radius;
    o.link.timeout = c.link.timeout;
    o.link.resume_on_recovery = c.link.resume_on_recovery;
    o.simplify = c.simplify;
    o.simplify_epsilon = c.simplify_epsilon;
    return o;
}

/// Adapts an API-level link to the module-level interface, converting the
/// module's waypoint span into API waypoints for the downstream implementation.
/// The API interface is fully qualified because the module base injects its own
/// `IAutopilotLink` name into this derived class.
class ModuleLinkAdapter final : public ugv::rth::IAutopilotLink {
public:
    using ApiLink = ::ugv::rth::api::IAutopilotLink;

    explicit ModuleLinkAdapter(ApiLink& out) noexcept : out_(out) {}

    void setMode(ugv::rth::AutopilotMode mode) override { out_.setMode(fromCore(mode)); }

    void uploadReturnPath(std::span<const ugv::rth::Waypoint> path) override {
        std::vector<Waypoint> api_path;
        api_path.reserve(path.size());
        for (const ugv::rth::Waypoint& w : path) {
            api_path.push_back(Waypoint{fromCore(w.pos), w.seq});
        }
        out_.uploadReturnPath(std::span<const Waypoint>(api_path));
    }

private:
    ApiLink& out_;
};

}  // namespace

Scalar distanceMeters(const GeoPoint& a, const GeoPoint& b) {
    return ugv::rth::geo::haversine(toCore(a), toCore(b));
}

struct MissionController::Impl {
    Impl(IAutopilotLink& link, const MissionConfig& cfg)
        : adapter(link), controller(adapter, toCore(cfg)) {}

    ModuleLinkAdapter adapter;                 // must outlive `controller`
    ugv::rth::MissionController controller;
};

MissionController::MissionController(IAutopilotLink& link, const MissionConfig& cfg)
    : impl_(std::make_unique<Impl>(link, cfg)) {}
MissionController::~MissionController() = default;
MissionController::MissionController(MissionController&&) noexcept = default;
MissionController& MissionController::operator=(MissionController&&) noexcept = default;

void MissionController::onHeartbeat(Stamp t) { impl_->controller.onHeartbeat(t); }
void MissionController::onPosition(const GeoPoint& p, Stamp t) {
    impl_->controller.onPosition(toCore(p), t);
}
void MissionController::tick(Stamp now) { impl_->controller.tick(now); }
void MissionController::forceReturn(Stamp now) { impl_->controller.forceReturn(now); }
void MissionController::reset() { impl_->controller.reset(); }

MissionMode MissionController::mode() const { return fromCore(impl_->controller.mode()); }
LinkState MissionController::linkState(Stamp now) const {
    return fromCore(impl_->controller.linkState(now));
}
bool MissionController::hasHome() const { return impl_->controller.hasHome(); }
Scalar MissionController::distanceToHome() const { return impl_->controller.distanceToHome(); }
std::size_t MissionController::routeSize() const { return impl_->controller.route().size(); }

std::vector<Waypoint> MissionController::returnPath() const {
    std::vector<Waypoint> out;
    const auto path = impl_->controller.returnPath();
    out.reserve(path.size());
    for (const ugv::rth::Waypoint& w : path) {
        out.push_back(Waypoint{fromCore(w.pos), w.seq});
    }
    return out;
}

std::vector<GeoPoint> MissionController::routePoints() const {
    std::vector<GeoPoint> out;
    const auto pts = impl_->controller.route().points();
    out.reserve(pts.size());
    for (const ugv::rth::GeoPoint& p : pts) {
        out.push_back(fromCore(p));
    }
    return out;
}

}  // namespace ugv::rth::api
