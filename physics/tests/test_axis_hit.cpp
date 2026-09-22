// Tests for the sweep-table helper. The bug being pinned here: the search used
// to keep the LAST axis above the threshold instead of the largest, so a
// second responding axis was dropped without a word -- and the probe's output
// is transcribed into docs/sitl_interface.md as measured fact.

#include "fdt/axis_hit.hpp"

#include <gtest/gtest.h>

namespace {

using fdt::probe::AxisHit;
using fdt::probe::interpret;
using fdt::probe::strongestAxis;

constexpr int kGyroThreshold = 300;

}  // namespace

TEST(StrongestAxis, ReportsNoHitWhenNothingClearsTheThreshold) {
  const AxisHit hit = strongestAxis({10, -299, 300}, kGyroThreshold);
  EXPECT_EQ(hit.axis, -1) << "the threshold is exclusive";
  EXPECT_EQ(hit.over, 0);
}

TEST(StrongestAxis, FindsASingleResponseOnEachAxis) {
  for (int axis = 0; axis < 3; ++axis) {
    std::array<int16_t, 3> v{0, 0, 0};
    v[static_cast<size_t>(axis)] = 940;
    const AxisHit hit = strongestAxis(v, kGyroThreshold);
    EXPECT_EQ(hit.axis, axis);
    EXPECT_EQ(hit.over, 1);
  }
}

TEST(StrongestAxis, FindsANegativeResponseByMagnitude) {
  const AxisHit hit = strongestAxis({0, -940, 0}, kGyroThreshold);
  EXPECT_EQ(hit.axis, 1);
  EXPECT_EQ(hit.over, 1);
}

TEST(StrongestAxis, KeepsTheLargestAxisNotTheLastOne) {
  // The regression. X is the real response; Z is a smaller cross-coupled
  // residual. The old loop returned Z purely because it came later.
  const AxisHit hit = strongestAxis({940, 0, 350}, kGyroThreshold);
  EXPECT_EQ(hit.axis, 0);
  EXPECT_EQ(hit.over, 2);
}

TEST(StrongestAxis, KeepsTheLargestAxisWhenTheLargestComesLast) {
  const AxisHit hit = strongestAxis({350, 0, -940}, kGyroThreshold);
  EXPECT_EQ(hit.axis, 2);
  EXPECT_EQ(hit.over, 2);
}

TEST(StrongestAxis, CountsEveryAxisThatResponded) {
  const AxisHit hit = strongestAxis({400, -500, 600}, kGyroThreshold);
  EXPECT_EQ(hit.axis, 2);
  EXPECT_EQ(hit.over, 3);
}

TEST(StrongestAxis, OnATieKeepsTheFirstAndStillFlagsTheAmbiguity) {
  const AxisHit hit = strongestAxis({940, -940, 0}, kGyroThreshold);
  EXPECT_EQ(hit.axis, 0);
  EXPECT_EQ(hit.over, 2) << "a tie is exactly the case that must not be reported as clean";
}

TEST(Interpret, NamesTheAxisAndTheSign) {
  EXPECT_EQ(interpret(0, {940, 0, 0}, kGyroThreshold), "fdm X -> BF X (same sign)");
  EXPECT_EQ(interpret(1, {0, 0, -940}, kGyroThreshold), "fdm Y -> BF Z (NEGATED)");
}

TEST(Interpret, SaysNoResponseWhenNothingMoved) {
  EXPECT_EQ(interpret(2, {0, 0, 0}, kGyroThreshold), "NO RESPONSE");
}

TEST(Interpret, MarksAnAmbiguousRowSoItIsNotTranscribed) {
  const std::string note = interpret(0, {940, 350, 0}, kGyroThreshold);
  EXPECT_NE(note.find("fdm X -> BF X (same sign)"), std::string::npos);
  EXPECT_NE(note.find("AMBIGUOUS"), std::string::npos);
  EXPECT_NE(note.find("2 axes responded"), std::string::npos);
}
