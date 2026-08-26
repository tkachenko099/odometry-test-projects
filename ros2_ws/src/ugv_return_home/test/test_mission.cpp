// Unit tests: the end-to-end return-home failsafe state machine.
#include <gtest/gtest.h>

#include <span>
#include <vector>

import ugv.rth;

using namespace ugv::rth;

namespace {

/// Records every command the controller issues to the "autopilot".
class MockLink final : public IAutopilotLink {
public:
    void setMode(AutopilotMode mode) override { modes.push_back(mode); }
    void uploadReturnPath(std::span<const Waypoint> path) override {
        ++uploads;
        last_path.assign(path.begin(), path.end());
    }

    std::vector<AutopilotMode> modes{};
    std::vector<Waypoint> last_path{};
    int uploads{0};
};

GeoPoint east(Scalar metres) noexcept { return {0.0, metres / geo::kEarthRadius, 0.0}; }

MissionConfig makeConfig() {
    MissionConfig c;
    c.route.min_record_distance = 2.0;
    c.route.arrival_radius = 3.0;
    c.link.timeout = 3.0;
    c.link.resume_on_recovery = true;
    c.simplify = false;
    return c;
}

}  // namespace

TEST(Mission, IdleUntilFirstFix) {
    MockLink link;
    MissionController ctrl{link, makeConfig()};
    EXPECT_EQ(ctrl.mode(), MissionMode::kIdle);
    ctrl.onHeartbeat(0.0);
    EXPECT_EQ(ctrl.mode(), MissionMode::kIdle);  // needs a position too
    ctrl.onPosition(east(0.0), 0.0);
    EXPECT_EQ(ctrl.mode(), MissionMode::kRecording);
}

TEST(Mission, LinkLossTriggersBacktrackAndArrivesHome) {
    MockLink link;
    MissionController ctrl{link, makeConfig()};

    // Drive out east with a healthy link.
    for (int m = 0; m <= 25; m += 5) {
        const Scalar t = static_cast<Scalar>(m) / 5.0;
        ctrl.onHeartbeat(t);
        ctrl.onPosition(east(static_cast<Scalar>(m)), t);
    }
    EXPECT_EQ(ctrl.mode(), MissionMode::kRecording);
    ASSERT_GE(ctrl.routeSize(), 2U);

    // Link goes silent; a later tick past the timeout trips the failsafe.
    ctrl.tick(9.0);
    EXPECT_EQ(ctrl.mode(), MissionMode::kReturning);
    EXPECT_EQ(link.uploads, 1);
    ASSERT_GE(link.last_path.size(), 2U);
    // First return waypoint = current (~25 m), last = home (~0 m).
    EXPECT_GT(link.last_path.front().pos.lon, link.last_path.back().pos.lon);
    ASSERT_GE(link.modes.size(), 2U);
    EXPECT_EQ(link.modes[0], AutopilotMode::kGuided);
    EXPECT_EQ(link.modes[1], AutopilotMode::kAuto);

    // Autopilot flies the vehicle back; feed decreasing positions.
    for (int m = 20; m >= 0; m -= 5) {
        ctrl.onPosition(east(static_cast<Scalar>(m)), 9.0 + static_cast<Scalar>(20 - m));
    }
    EXPECT_EQ(ctrl.mode(), MissionMode::kHome);
    EXPECT_EQ(link.modes.back(), AutopilotMode::kHold);
    EXPECT_LE(ctrl.distanceToHome(), makeConfig().route.arrival_radius);
}

TEST(Mission, HeartbeatRecoveryAbortsReturn) {
    MockLink link;
    MissionController ctrl{link, makeConfig()};
    ctrl.onHeartbeat(0.0);
    ctrl.onPosition(east(0.0), 0.0);
    ctrl.onPosition(east(10.0), 1.0);
    ctrl.tick(5.0);  // link lost -> returning
    ASSERT_EQ(ctrl.mode(), MissionMode::kReturning);

    ctrl.onHeartbeat(6.0);  // operator regains control
    EXPECT_EQ(ctrl.mode(), MissionMode::kRecording);
    EXPECT_EQ(link.modes.back(), AutopilotMode::kManual);
}

TEST(Mission, ForceReturnTriggersManually) {
    MockLink link;
    MissionController ctrl{link, makeConfig()};
    ctrl.onHeartbeat(0.0);
    ctrl.onPosition(east(0.0), 0.0);
    ctrl.onPosition(east(10.0), 1.0);
    EXPECT_EQ(ctrl.mode(), MissionMode::kRecording);
    ctrl.forceReturn(2.0);
    EXPECT_EQ(ctrl.mode(), MissionMode::kReturning);
    EXPECT_EQ(link.uploads, 1);
}

TEST(Mission, ResetClearsEverything) {
    MockLink link;
    MissionController ctrl{link, makeConfig()};
    ctrl.onHeartbeat(0.0);
    ctrl.onPosition(east(0.0), 0.0);
    ctrl.onPosition(east(10.0), 1.0);
    ctrl.reset();
    EXPECT_EQ(ctrl.mode(), MissionMode::kIdle);
    EXPECT_EQ(ctrl.routeSize(), 0U);
    EXPECT_FALSE(ctrl.hasHome());
}
