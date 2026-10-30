// test_mycobot.cpp - GoogleTests for the hardware-independent parts of mycobot.hpp:
// poses and limits, stepToward(), minHeightMm(), the temperature checks and the PID bounds.
// No serial port is opened.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <stdexcept>

#include "mycobot.hpp"

namespace mc = mycobot;

// ---- Poses and limits ----------------------------------------------------------

TEST(Poses, InsideJointLimits) {
    for (const auto& pose : {mc::ZERO_ANGLES, mc::PARK_ANGLES}) {
        for (size_t i = 0; i < pose.size(); ++i) {
            EXPECT_GE(pose[i], mc::JOINT_LIMITS[i].min) << "J" << i + 1;
            EXPECT_LE(pose[i], mc::JOINT_LIMITS[i].max) << "J" << i + 1;
        }
    }
}

TEST(Poses, AboveTheTable) {
    EXPECT_GE(mc::minHeightMm(mc::ZERO_ANGLES), mc::TABLE_MARGIN_MM);
    EXPECT_GE(mc::minHeightMm(mc::PARK_ANGLES), mc::TABLE_MARGIN_MM);
}

// ---- minHeightMm ---------------------------------------------------------------

TEST(MinHeight, StraightUpIsTheHighest) {
    const double up = mc::minHeightMm(mc::ZERO_ANGLES);
    EXPECT_GT(up, mc::minHeightMm(mc::PARK_ANGLES));
    EXPECT_GT(up, mc::minHeightMm({0, 90, 0, 0, 0, 0}));
}

TEST(MinHeight, J1DoesNotChangeTheHeight) {
    // J1 turns about the vertical axis.
    const mc::Angles pose = {0, 60, -30, 20, 10, 0};
    for (double j1 : {-150.0, -45.0, 90.0, 160.0}) {
        mc::Angles turned = pose;
        turned[0] = j1;
        EXPECT_NEAR(mc::minHeightMm(turned), mc::minHeightMm(pose), 1e-6) << "J1 " << j1;
    }
}

TEST(MinHeight, LeaningIntoTheTableIsBelowTheMargin) {
    EXPECT_LT(mc::minHeightMm({0, -130, 0, 0, 0, 0}), mc::TABLE_MARGIN_MM);
}

// ---- stepToward ----------------------------------------------------------------

TEST(StepToward, NothingWithinTolerance) {
    const mc::Angles near = {1, -1, 0.5, 0, 0, 1.5};
    EXPECT_FALSE(mc::stepToward(near, mc::ZERO_ANGLES, 10.0, 2.0).has_value());
}

TEST(StepToward, BoundedAndSynchronised) {
    const mc::Angles current = {40, -20, 0, 10, 0, 0};
    const auto step = mc::stepToward(current, mc::ZERO_ANGLES, 10.0, 2.0);
    ASSERT_TRUE(step.has_value());
    // The worst joint (J1, 40 deg) moves by the lead; the others by the same fraction.
    EXPECT_NEAR((*step)[0], 30.0, 0.05);
    EXPECT_NEAR((*step)[1], -15.0, 0.05);
    EXPECT_NEAR((*step)[3], 7.5, 0.05);
    EXPECT_DOUBLE_EQ((*step)[2], 0.0);
}

TEST(StepToward, ShortDistanceArrivesInOneStep) {
    const mc::Angles current = {5, 0, 0, 0, 0, 0};
    const auto step = mc::stepToward(current, mc::ZERO_ANGLES, 10.0, 2.0);
    ASSERT_TRUE(step.has_value());
    EXPECT_EQ(*step, mc::ZERO_ANGLES);
}

TEST(StepToward, RoundsToATenthOfADegree) {
    const mc::Angles current = {33, 0, 0, 0, 0, 0};
    const auto step = mc::stepToward(current, mc::ZERO_ANGLES, 10.0, 2.0);
    ASSERT_TRUE(step.has_value());
    for (double a : *step) EXPECT_DOUBLE_EQ(a, std::round(a * 10.0) / 10.0);
}

// ---- Temperatures --------------------------------------------------------------

TEST(TempLevel, Thresholds) {
    EXPECT_EQ(mc::MyCobot280::tempLevel(mc::TEMP_WARM_C - 1), mc::TempLevel::Normal);
    EXPECT_EQ(mc::MyCobot280::tempLevel(mc::TEMP_WARM_C), mc::TempLevel::Warm);
    EXPECT_EQ(mc::MyCobot280::tempLevel(mc::TEMP_HOT_C - 1), mc::TempLevel::Warm);
    EXPECT_EQ(mc::MyCobot280::tempLevel(mc::TEMP_HOT_C), mc::TempLevel::Hot);
}

TEST(EvaluateTemps, HottestJointSetsTheLevel) {
    const auto check = mc::evaluateTemps({30, 31, 40, 66, 50, 30});
    EXPECT_EQ(check.hottestJoint, 4);
    EXPECT_EQ(check.level, mc::TempLevel::Hot);
}

TEST(EvaluateTemps, TieGoesToTheFirstJoint) {
    const auto check = mc::evaluateTemps({30, 56, 30, 56, 30, 30});
    EXPECT_EQ(check.hottestJoint, 2);
    EXPECT_EQ(check.level, mc::TempLevel::Warm);
}

TEST(TempColour, GreenYellowRed) {
    const auto g = mc::tempColour(mc::TempLevel::Normal);
    const auto y = mc::tempColour(mc::TempLevel::Warm);
    const auto r = mc::tempColour(mc::TempLevel::Hot);
    EXPECT_EQ((std::array<int, 3>{g.r, g.g, g.b}), (std::array<int, 3>{0, 255, 0}));
    EXPECT_EQ((std::array<int, 3>{y.r, y.g, y.b}), (std::array<int, 3>{255, 255, 0}));
    EXPECT_EQ((std::array<int, 3>{r.r, r.g, r.b}), (std::array<int, 3>{255, 0, 0}));
}

// ---- PID bounds ----------------------------------------------------------------

TEST(PidBounds, FactoryValuesAreAccepted) {
    for (int j = 1; j <= 6; ++j)
        EXPECT_NO_THROW(mc::MyCobot280::checkPidBounds(j, mc::FACTORY_PID[j - 1]));
}

TEST(PidBounds, OutOfRangeIsRefused) {
    const mc::Pid f = mc::FACTORY_PID[0];  // J1: P 15, I 0, D 20
    EXPECT_THROW(mc::MyCobot280::checkPidBounds(1, {-1, 0, f.d}), std::invalid_argument);
    EXPECT_THROW(mc::MyCobot280::checkPidBounds(1, {23, 0, f.d}),
                 std::invalid_argument);  // max P 22
    EXPECT_THROW(mc::MyCobot280::checkPidBounds(1, {f.p, mc::PID_MAX_I + 1, f.d}),
                 std::invalid_argument);
    EXPECT_THROW(mc::MyCobot280::checkPidBounds(1, {f.p, 0, 31}),
                 std::invalid_argument);  // max D 30
    EXPECT_NO_THROW(mc::MyCobot280::checkPidBounds(1, {22, mc::PID_MAX_I, 30}));
}

TEST(PidBounds, JointOutsideOneToSixIsRefused) {
    EXPECT_THROW(mc::MyCobot280::checkPidBounds(0, mc::FACTORY_PID[0]), std::invalid_argument);
    EXPECT_THROW(mc::MyCobot280::checkPidBounds(7, mc::FACTORY_PID[0]), std::invalid_argument);
}
