#include <gtest/gtest.h>
#include <object_detector/cores/tracking.hpp>
#include <cmath>
#include <vector>
#include <utility>

namespace {
interfaces::Objects frame(double t, const std::vector<std::pair<double,double>>& xy) {
    interfaces::Objects msg;
    msg.header.stamp.fromSec(t);
    for (const auto& point : xy) {
        interfaces::Object obj;
        obj.position.x = point.first;
        obj.position.y = point.second;
        msg.objects.push_back(obj);
    }
    return msg;
}
}

TEST(Tracking, StableIdForStationaryObject) {
    object_detector::Tracking tracking;
    const auto first = tracking.process(frame(1.0, {{10.0, 5.0}}));
    const auto second = tracking.process(frame(1.1, {{10.0, 5.0}}));
    EXPECT_EQ(first.objects[0].id, second.objects[0].id);
    EXPECT_FALSE(first.objects[0].velocity_available);
    EXPECT_TRUE(second.objects[0].velocity_available);
    EXPECT_NEAR(second.objects[0].velocity.x, 0.0, 0.01);
    EXPECT_NEAR(second.objects[0].velocity.y, 0.0, 0.01);
}

TEST(Tracking, EstimatesConstantVelocityAndDirection) {
    object_detector::Tracking tracking;
    interfaces::Objects output;
    unsigned initial_id = 0;
    for (int i=0; i<35; ++i) {
        const double t = 1.0 + 0.1*i;
        output = tracking.process(frame(t, {{2.0+0.3*i, 4.0+0.1*i}}));
        if (i==0) initial_id = output.objects[0].id;
        EXPECT_EQ(output.objects[0].id, initial_id);
    }
    EXPECT_TRUE(output.objects[0].velocity_available);
    EXPECT_NEAR(output.objects[0].velocity.x, 3.0, 0.2);
    EXPECT_NEAR(output.objects[0].velocity.y, 1.0, 0.2);
    EXPECT_NEAR(std::atan2(output.objects[0].velocity.y, output.objects[0].velocity.x), std::atan2(1.0,3.0), 0.08);
}

TEST(Tracking, TwoObjectsKeepIdsWhenDetectionOrderChanges) {
    object_detector::Tracking tracking;
    const auto before = tracking.process(frame(1.0, {{0, 0}, {10, 0}}));
    const auto after = tracking.process(frame(1.1, {{10.1, 0}, {0.1, 0}}));
    EXPECT_EQ(after.objects[0].id, before.objects[1].id);
    EXPECT_EQ(after.objects[1].id, before.objects[0].id);
}

TEST(Tracking, NewObjectOutsideGatingDistanceGetsNewId) {
    object_detector::Tracking tracking;
    const auto before = tracking.process(frame(1.0, {{0, 0}}));
    const auto after = tracking.process(frame(1.1, {{3.0, 0}}));
    EXPECT_NE(after.objects[0].id, before.objects[0].id);
    EXPECT_FALSE(after.objects[0].velocity_available);
}

TEST(Tracking, MissingDetectionDropsTrackInCurrentPrototype) {
    object_detector::Tracking tracking;
    const auto before = tracking.process(frame(1.0, {{0, 0}}));
    const auto empty = tracking.process(frame(1.1, {}));
    const auto after = tracking.process(frame(1.2, {{0, 0}}));
    EXPECT_TRUE(empty.objects.empty());
    EXPECT_NE(after.objects[0].id, before.objects[0].id);
}

TEST(Tracking, TimestampStartingAtZeroStillUsesElapsedTime) {
    object_detector::Tracking tracking;
    const auto first = tracking.process(frame(0.0, {{0.0, 0.0}}));
    const auto second = tracking.process(frame(0.1, {{0.3, 0.0}}));
    EXPECT_EQ(first.objects[0].id, second.objects[0].id);
    EXPECT_TRUE(second.objects[0].velocity_available);
    EXPECT_TRUE(second.objects[0].velocity.x > 0.0);
}

TEST(Tracking, CrossingPathsRetainIdsAfterVelocityConverges) {
    object_detector::Tracking tracking;
    unsigned first_id = 0;
    unsigned second_id = 0;
    for (int i=0; i<35; ++i) {
        auto out = tracking.process(frame(1.0+0.1*i,
                                      {{0.0, -2.0+0.2*i}, {0.4, 2.0-0.2*i}}));
        if (i==0) {first_id=out.objects[0].id; second_id=out.objects[1].id;}
        EXPECT_EQ(out.objects[0].id, first_id);
        EXPECT_EQ(out.objects[1].id, second_id);
    }
}
