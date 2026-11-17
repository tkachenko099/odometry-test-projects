// Unit tests: operator-link heartbeat monitor.
#include <gtest/gtest.h>

import ugv.rth;

using namespace ugv::rth;

TEST(LinkMonitor, UninitialisedUntilFirstBeat) {
    LinkMonitor mon{LinkConfig{.timeout = 3.0, .resume_on_recovery = true}};
    EXPECT_EQ(mon.state(0.0), LinkState::kUninitialised);
    EXPECT_FALSE(mon.connected(0.0));
}

TEST(LinkMonitor, ConnectedWithinTimeout) {
    LinkMonitor mon{LinkConfig{.timeout = 3.0, .resume_on_recovery = true}};
    mon.heartbeat(10.0);
    EXPECT_EQ(mon.state(11.0), LinkState::kConnected);
    EXPECT_TRUE(mon.connected(12.99));
}

TEST(LinkMonitor, LostAfterTimeout) {
    LinkMonitor mon{LinkConfig{.timeout = 3.0, .resume_on_recovery = true}};
    mon.heartbeat(10.0);
    EXPECT_EQ(mon.state(13.5), LinkState::kLost);
    EXPECT_TRUE(mon.lost(20.0));
    EXPECT_NEAR(mon.sinceLastBeat(15.0), 5.0, 1e-9);
}

TEST(LinkMonitor, RecoveryRestoresConnected) {
    LinkMonitor mon{LinkConfig{.timeout = 2.0, .resume_on_recovery = true}};
    mon.heartbeat(0.0);
    EXPECT_TRUE(mon.lost(5.0));
    mon.heartbeat(5.0);
    EXPECT_TRUE(mon.connected(6.0));
}
