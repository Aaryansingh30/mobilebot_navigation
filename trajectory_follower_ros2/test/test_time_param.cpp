// Purpose: Unit test for time-parameterization monotonicity.
#include "trajectory_follower_ros2/trajectory_time_param.hpp"

#include <gtest/gtest.h>

TEST(TimeParam, MonotonicAndNonNegative) {
    // Simple arc-length sequence should produce non-decreasing time stamps.
    std::vector<double> s{0.0, 0.5, 1.0, 2.0};
    auto t = compute_time_stamps(s, 0.5, 0.5);
    ASSERT_EQ(t.size(), s.size());
    for (size_t i = 1; i < t.size(); ++i) {
        EXPECT_GE(t[i], t[i - 1]);
    }
    EXPECT_GE(t.front(), 0.0);
}
