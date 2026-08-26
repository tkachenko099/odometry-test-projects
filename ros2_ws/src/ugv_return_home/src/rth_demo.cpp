// Standalone (no-ROS) demonstration of the return-home failsafe.
//
// Drives a simulated UGV along an L-shaped route under operator control, drops
// the datalink, and lets the MissionController backtrack the vehicle home via a
// simulated MAVLink autopilot. Deterministic; prints a step-by-step trace and,
// when given a path argument, writes a JSON run log for plotting:
//
//     rth_demo [run.json]
#include "ugv_return_home/ugv_return_home.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace api = ugv::rth::api;
constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kR = 6'371'000.0;

// Reference launch point (Kyiv). Local (north,east) metres -> geodetic radians.
constexpr double kLat0 = 50.4501 * kDeg;
constexpr double kLon0 = 30.5234 * kDeg;

[[nodiscard]] api::GeoPoint fromMeters(double north, double east) noexcept {
    return {kLat0 + north / kR, kLon0 + east / (kR * std::cos(kLat0)), 0.0};
}

/// Inverse of fromMeters: geodetic -> local ENU metres relative to launch.
[[nodiscard]] std::pair<double, double> toMeters(const api::GeoPoint& p) noexcept {
    return {(p.lat - kLat0) * kR, (p.lon - kLon0) * kR * std::cos(kLat0)};  // (north, east)
}

[[nodiscard]] const char* modeName(api::AutopilotMode m) noexcept {
    switch (m) {
        case api::AutopilotMode::kManual: return "MANUAL";
        case api::AutopilotMode::kHold: return "HOLD";
        case api::AutopilotMode::kGuided: return "GUIDED";
        case api::AutopilotMode::kAuto: return "AUTO";
        case api::AutopilotMode::kReturnToLaunch: return "RTL";
        default: return "UNKNOWN";
    }
}

[[nodiscard]] const char* missionName(api::MissionMode m) noexcept {
    switch (m) {
        case api::MissionMode::kIdle: return "IDLE";
        case api::MissionMode::kRecording: return "RECORDING";
        case api::MissionMode::kReturning: return "RETURNING";
        case api::MissionMode::kHome: return "HOME";
        default: return "?";
    }
}

/// One logged trajectory sample (true vehicle pose + mission state).
struct Sample {
    double t;
    double north;
    double east;
    const char* state;
};

/// Simulated MAVLink autopilot backend.
class SimulatedAutopilot final : public api::IAutopilotLink {
public:
    void setMode(api::AutopilotMode mode) override {
        mode_ = mode;
        std::printf("    [MAVLink] SET_MODE -> %s\n", modeName(mode));
    }
    void uploadReturnPath(std::span<const api::Waypoint> path) override {
        mission_.assign(path.begin(), path.end());
        std::printf("    [MAVLink] MISSION_COUNT=%zu uploaded (current -> home)\n", path.size());
    }
    [[nodiscard]] const std::vector<api::Waypoint>& mission() const noexcept { return mission_; }
    [[nodiscard]] api::AutopilotMode mode() const noexcept { return mode_; }

private:
    std::vector<api::Waypoint> mission_{};
    api::AutopilotMode mode_{api::AutopilotMode::kManual};
};

/// Serialise the run to JSON (no external deps) for the Python plotter.
void writeJson(const std::string& path, const std::vector<Sample>& samples,
               const std::vector<std::pair<double, double>>& trail,
               const std::vector<std::pair<double, double>>& waypoints, double drop_t) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        std::printf("  [warn] could not open %s for writing\n", path.c_str());
        return;
    }
    const auto arr2 = [&](const std::vector<std::pair<double, double>>& v) {
        std::fputc('[', f);
        for (std::size_t i = 0; i < v.size(); ++i)
            std::fprintf(f, "%s[%.4f,%.4f]", i ? "," : "", v[i].first, v[i].second);
        std::fputc(']', f);
    };
    std::fprintf(f, "{\n  \"link_loss_t\": %.3f,\n  \"trail\": ", drop_t);
    arr2(trail);
    std::fprintf(f, ",\n  \"waypoints\": ");
    arr2(waypoints);
    std::fprintf(f, ",\n  \"samples\": [");
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const Sample& s = samples[i];
        std::fprintf(f, "%s\n    {\"t\":%.3f,\"north\":%.4f,\"east\":%.4f,\"state\":\"%s\"}",
                     i ? "," : "", s.t, s.north, s.east, s.state);
    }
    std::fprintf(f, "\n  ]\n}\n");
    std::fclose(f);
    std::printf("  run log written to %s (%zu samples)\n", path.c_str(), samples.size());
}

}  // namespace

int main(int argc, char** argv) {
    api::MissionConfig cfg;
    cfg.route.min_record_distance = 3.0;
    cfg.route.arrival_radius = 2.5;
    cfg.link.timeout = 2.0;
    cfg.link.resume_on_recovery = false;
    cfg.simplify = true;
    cfg.simplify_epsilon = 1.0;

    SimulatedAutopilot autopilot;
    api::MissionController ctrl{autopilot, cfg};
    std::vector<Sample> samples;

    std::puts("== Phase 1: operator drives an L-shaped route (link healthy) ==");
    double t = 0.0;
    const double dt = 0.1;  // 10 Hz
    std::vector<std::pair<double, double>> route;          // (north, east) waypoints
    for (double e = 0.0; e <= 50.0; e += 0.5) route.emplace_back(0.0, e);   // east leg
    for (double n = 0.5; n <= 50.0; n += 0.5) route.emplace_back(n, 50.0);  // north leg
    for (const auto& [north, east] : route) {
        ctrl.onHeartbeat(t);
        ctrl.onPosition(fromMeters(north, east), t);
        samples.push_back({t, north, east, missionName(ctrl.mode())});
        t += dt;
    }
    std::printf("  recorded %zu breadcrumbs, driven ~%.0f m, mode=%s\n", ctrl.routeSize(),
                100.0, missionName(ctrl.mode()));

    std::puts("== Phase 2: operator link drops ==");
    const double drop_t = t;
    double cur_n = 50.0, cur_e = 50.0;  // vehicle idles at last position
    for (int i = 0; i < 40; ++i) {  // no heartbeats; tick forward
        t += dt;
        ctrl.tick(t);
        samples.push_back({t, cur_n, cur_e, missionName(ctrl.mode())});
        if (ctrl.mode() == api::MissionMode::kReturning) {
            std::printf("  link silent for %.1fs -> FAILSAFE at t=%.1f\n", t - drop_t, t);
            break;
        }
    }
    std::printf("  return mission has %zu waypoints (autopilot in %s)\n",
                autopilot.mission().size(), modeName(autopilot.mode()));

    std::puts("== Phase 3: autopilot backtracks the vehicle home (smooth follow) ==");
    const double v = 5.0;                 // [m/s] backtrack speed
    const double step = v * dt;           // metres per tick
    const double wob_amp = 0.7;           // [m] emulated tracking wobble (pure-pursuit)
    const double wob_period = 22.0;       // [m] wobble wavelength
    double s_acc = 0.0;                    // distance travelled during backtrack
    for (const api::Waypoint& wp : autopilot.mission()) {
        const auto [tn, te] = toMeters(wp.pos);
        double seg_travel = 0.0;           // distance since this leg started
        // March toward the waypoint at constant speed, sampling each tick, with a
        // small perpendicular wobble so the driven path deviates realistically.
        for (;;) {
            const double dn = tn - cur_n, de = te - cur_e;
            const double dist = std::hypot(dn, de);
            if (dist <= step) { cur_n = tn; cur_e = te; break; }
            cur_n += step * dn / dist;
            cur_e += step * de / dist;
            s_acc += step;
            seg_travel += step;
            // Taper the wobble to zero near each leg's endpoints so the driven
            // path stays continuous at corners (no perpendicular sign flip).
            const double env = std::min({seg_travel / 4.0, dist / 4.0, 1.0});
            const double off =
                wob_amp * env * std::sin(2.0 * std::numbers::pi * s_acc / wob_period);
            const double pn = cur_n + (-de / dist) * off;  // left-normal offset
            const double pe = cur_e + (dn / dist) * off;
            t += dt;
            ctrl.onPosition(fromMeters(pn, pe), t);
            samples.push_back({t, pn, pe, missionName(ctrl.mode())});
        }
        t += dt;
        ctrl.onPosition(wp.pos, t);
        samples.push_back({t, cur_n, cur_e, missionName(ctrl.mode())});
        if (ctrl.mode() == api::MissionMode::kHome) break;
    }
    std::printf("  final mode=%s, distance-to-home=%.2f m, autopilot=%s\n",
                missionName(ctrl.mode()), ctrl.distanceToHome(), modeName(autopilot.mode()));

    if (argc > 1) {
        std::vector<std::pair<double, double>> trail, wpts;
        for (const api::GeoPoint& p : ctrl.routePoints()) trail.push_back(toMeters(p));
        for (const api::Waypoint& w : autopilot.mission()) wpts.push_back(toMeters(w.pos));
        writeJson(argv[1], samples, trail, wpts, drop_t);
    }

    const bool ok = ctrl.mode() == api::MissionMode::kHome;
    std::puts(ok ? "\nRESULT: vehicle returned home successfully." : "\nRESULT: FAILED.");
    return ok ? 0 : 1;
}
