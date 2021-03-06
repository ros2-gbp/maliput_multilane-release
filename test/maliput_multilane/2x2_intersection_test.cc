// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/branch_point.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/road_geometry.h>
#include <maliput/common/filesystem.h>

#include "maliput_multilane/builder.h"
#include "maliput_multilane/loader.h"

namespace maliput {
namespace multilane {
namespace {

using api::InertialPosition;
using api::JunctionId;
using api::Lane;
using api::LaneEnd;
using api::LaneId;
using api::LanePosition;
using api::LanePositionResult;
using api::RBounds;
using api::RoadGeometry;

constexpr double kStartDistance{59.375};
constexpr double kEntryDistance{9.375};
constexpr double kMidDistance{kStartDistance / 2.0 + kEntryDistance};
constexpr double kLaneWidth{3.75};
constexpr double kHalfLaneWidth{kLaneWidth / 2.0};
constexpr double kApproachLength{50.0};
constexpr double kIntersectionWidth{kEntryDistance * 2.0};

class Test2x2Intersection : public ::testing::Test {
 protected:
  virtual void SetUp() {
    static const char* const kFileName = "/2x2_intersection.yaml";
    const std::string resource_path = DEF_MULTILANE_RESOURCE_ROOT;
    ASSERT_TRUE(!resource_path.empty());
    dut_ = LoadFile(multilane::BuilderFactory(), resource_path + kFileName);
  }

  std::unique_ptr<const RoadGeometry> dut_;
};

TEST_F(Test2x2Intersection, CheckRoadGeometryProperties) {
  EXPECT_NE(dut_, nullptr);
  EXPECT_EQ(dut_->id().string(), "basic_two_lane_x_intersection");
  EXPECT_EQ(dut_->num_junctions(), 5);
  EXPECT_EQ(dut_->num_branch_points(), 16);
  EXPECT_NEAR(dut_->linear_tolerance(), 0.01, 1e-6);
  EXPECT_NEAR(dut_->angular_tolerance(), 0.5 * M_PI / 180., 1e-6);
  EXPECT_NEAR(dut_->scale_length(), 1, 1e-6);
  EXPECT_TRUE(dut_->CheckInvariants().empty());
  EXPECT_EQ(dut_->ById().GetJunction(JunctionId("j:intersection"))->num_segments(), 10);
}

TEST_F(Test2x2Intersection, CheckLaneNamesAndPositions) {
  struct ExpectedLane {
    LaneId id;
    double length;
    // These are positions that should fall within the lane.
    std::vector<InertialPosition> inertial_positions;
  };
  const std::vector<ExpectedLane> test_cases{
      // Segment west of intersection.
      // East-bound lane.
      {LaneId("l:w_segment_0"),
       kApproachLength,
       {InertialPosition(-kStartDistance, -kHalfLaneWidth, 0), InertialPosition(-kMidDistance, -kHalfLaneWidth, 0),
        InertialPosition(-kEntryDistance, -kLaneWidth, 0)}},
      // West-bound lane.
      {LaneId("l:w_segment_1"),
       kApproachLength,
       {InertialPosition(-kStartDistance, kHalfLaneWidth, 0), InertialPosition(-kMidDistance, kHalfLaneWidth, 0),
        InertialPosition(-kEntryDistance, kHalfLaneWidth, 0)}},

      // Segment east of intersection.
      // East-bound lane.
      {LaneId("l:e_segment_0"),
       kApproachLength,
       {InertialPosition(kEntryDistance, -kHalfLaneWidth, 0), InertialPosition(kMidDistance, -kHalfLaneWidth, 0),
        InertialPosition(kStartDistance, -kHalfLaneWidth, 0)}},
      // West-bound lane.
      {LaneId("l:e_segment_1"),
       kApproachLength,
       {InertialPosition(kEntryDistance, kHalfLaneWidth, 0), InertialPosition(kMidDistance, kHalfLaneWidth, 0),
        InertialPosition(kStartDistance, kHalfLaneWidth, 0)}},

      // Segment north of intersection.
      // North-bound lane.
      {LaneId("l:n_segment_0"),
       kApproachLength,
       {InertialPosition(kHalfLaneWidth, kEntryDistance, 0), InertialPosition(kHalfLaneWidth, kMidDistance, 0),
        InertialPosition(kHalfLaneWidth, kStartDistance, 0)}},
      // South-bound lane.
      {LaneId("l:n_segment_1"),
       kApproachLength,
       {InertialPosition(-kHalfLaneWidth, kEntryDistance, 0), InertialPosition(-kHalfLaneWidth, kMidDistance, 0),
        InertialPosition(-kHalfLaneWidth, kStartDistance, 0)}},

      // Segment south of intersection.
      // North-bound lane.
      {LaneId("l:s_segment_0"),
       kApproachLength,
       {InertialPosition(kHalfLaneWidth, -kEntryDistance, 0), InertialPosition(kHalfLaneWidth, -kMidDistance, 0),
        InertialPosition(kHalfLaneWidth, -kStartDistance, 0)}},
      // South-bound lane.
      {LaneId("l:s_segment_1"),
       kApproachLength,
       {InertialPosition(-kHalfLaneWidth, -kEntryDistance, 0), InertialPosition(-kHalfLaneWidth, -kMidDistance, 0),
        InertialPosition(-kHalfLaneWidth, -kStartDistance, 0)}},

      // East/West segment inside the intersection.
      // East-bound lane.
      {LaneId("l:ew_intersection_segment_0"),
       kIntersectionWidth,
       {InertialPosition(0, -kHalfLaneWidth, 0), InertialPosition(8, -kHalfLaneWidth, 0),
        InertialPosition(-2.33, -2.33, 0), InertialPosition(2.33, -2.33, 0)}},
      // West-bound lane.
      {LaneId("l:ew_intersection_segment_1"),
       kIntersectionWidth,
       {InertialPosition(0, kHalfLaneWidth, 0), InertialPosition(8, kHalfLaneWidth, 0), InertialPosition(2.33, 2.33, 0),
        InertialPosition(-2.33, 2.33, 0)}},

      // North/South segment inside the intersection.
      // North-bound lane.
      {LaneId("l:ns_intersection_segment_0"),
       kIntersectionWidth,
       {InertialPosition(kHalfLaneWidth, 0, 0), InertialPosition(kHalfLaneWidth, -8, 0),
        InertialPosition(2.33, -2.33, 0), InertialPosition(2.33, 2.33, 0)}},
      // South-bound lane.
      {LaneId("l:ns_intersection_segment_1"),
       kIntersectionWidth,
       {InertialPosition(0, -kHalfLaneWidth, 0), InertialPosition(-kHalfLaneWidth, 8, 0),
        InertialPosition(-2.33, -2.33, 0), InertialPosition(-2.33, 2.33, 0)}},

      // Right turn lanes.
      {LaneId("l:east_right_turn_segment_0"),
       M_PI_2 * 7.5,
       {InertialPosition(-kEntryDistance, -kHalfLaneWidth, 0), InertialPosition(-4.8, -4.8, 0),
        InertialPosition(-kHalfLaneWidth, -kEntryDistance, 0)}},
      {LaneId("l:west_right_turn_segment_0"),
       M_PI_2 * 7.5,
       {InertialPosition(kEntryDistance, kHalfLaneWidth, 0), InertialPosition(4.8, 4.8, 0),
        InertialPosition(kHalfLaneWidth, kEntryDistance, 0)}},
      {LaneId("l:north_right_turn_segment_0"),
       M_PI_2 * 7.5,
       {InertialPosition(kHalfLaneWidth, -kEntryDistance, 0), InertialPosition(4.8, -4.8, 0),
        InertialPosition(kEntryDistance, -kHalfLaneWidth, 0)}},
      {LaneId("l:south_right_turn_segment_0"),
       M_PI_2 * 7.5,
       {InertialPosition(-kHalfLaneWidth, kEntryDistance, 0), InertialPosition(-4.8, 4.8, 0),
        InertialPosition(-kEntryDistance, kHalfLaneWidth, 0)}},

      // Left turn lanes.
      {LaneId("l:east_left_turn_segment_0"),
       M_PI_2 * 11.25,
       {InertialPosition(-kEntryDistance, -kHalfLaneWidth, 0), InertialPosition(-1.66, 0.64, 0),
        InertialPosition(kHalfLaneWidth, kEntryDistance, 0)}},
      {LaneId("l:west_left_turn_segment_0"),
       M_PI_2 * 11.25,
       {InertialPosition(kEntryDistance, kHalfLaneWidth, 0), InertialPosition(1.66, -0.64, 0),
        InertialPosition(-kHalfLaneWidth, -kEntryDistance, 0)}},
      {LaneId("l:north_left_turn_segment_0"),
       M_PI_2 * 11.25,
       {InertialPosition(kHalfLaneWidth, -kEntryDistance, 0), InertialPosition(0.64, -1.66, 0),
        InertialPosition(-kEntryDistance, kHalfLaneWidth, 0)}},
      {LaneId("l:south_left_turn_segment_0"),
       M_PI_2 * 11.25,
       {InertialPosition(-kHalfLaneWidth, kEntryDistance, 0), InertialPosition(-0.64, 1.66, 0),
        InertialPosition(kEntryDistance, -kHalfLaneWidth, 0)}},
  };
  for (const auto& test_case : test_cases) {
    const Lane* lane = dut_->ById().GetLane(test_case.id);
    EXPECT_NE(lane, nullptr);
    EXPECT_NEAR(lane->length(), test_case.length, dut_->linear_tolerance());
    for (const auto& inertial_position : test_case.inertial_positions) {
      const LanePositionResult result = lane->ToLanePosition(inertial_position);
      EXPECT_NEAR(result.distance, 0, dut_->linear_tolerance());
      const RBounds lane_bounds = lane->lane_bounds(result.lane_position.s());
      EXPECT_TRUE(lane_bounds.min() <= result.lane_position.r());
      EXPECT_TRUE(lane_bounds.max() >= result.lane_position.r());
    }
  }
}

TEST_F(Test2x2Intersection, CheckBranches) {
  struct BranchInfo {
    LaneId id;
    LaneEnd::Which which_end;
  };
  // Tests the lanes that lead up to the intersection. These lanes have three
  // ongoing branches: forward, left turn, and right turn. They have only one
  // confluent branch, which is the lane itself.
  const std::vector<BranchInfo> outside_intersection_test_cases{
      // Segment west of intersection.
      {LaneId("l:w_segment_0"), LaneEnd::kFinish},  // East bound lane.
      {LaneId("l:w_segment_1"), LaneEnd::kFinish},  // West-bound lane.
      // Segment south of intersection.
      {LaneId("l:s_segment_0"), LaneEnd::kFinish},  // North-bound lane.
      {LaneId("l:s_segment_1"), LaneEnd::kFinish},  // South-bound lane.
      // Segment east of intersection.
      {LaneId("l:e_segment_1"), LaneEnd::kStart},  // West-bound lane.
      {LaneId("l:e_segment_0"), LaneEnd::kStart},  // East-bound lane.
      // Segment north of intersection.
      {LaneId("l:n_segment_1"), LaneEnd::kStart},  // South-bound lane.
      {LaneId("l:n_segment_0"), LaneEnd::kStart},  // North-bound lane.
  };
  for (const auto& test_case : outside_intersection_test_cases) {
    const Lane* lane = dut_->ById().GetLane(test_case.id);
    EXPECT_NE(lane, nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(test_case.which_end)->size(), 3);
    EXPECT_EQ(lane->GetConfluentBranches(test_case.which_end)->size(), 1);
  }
  // Tests the lanes within the intersection. For both ends of these lanes, they
  // only have one ongoing branch, which is to the lane that either leads away
  // from or towards the intersection. They have three confluent branches:
  // forward, left turn, and right turn.
  const std::vector<LaneId> inside_intersection_test_cases{
      {LaneId("l:ew_intersection_segment_0")},  {LaneId("l:ew_intersection_segment_1")},
      {LaneId("l:ns_intersection_segment_0")},  {LaneId("l:ns_intersection_segment_1")},
      {LaneId("l:east_right_turn_segment_0")},  {LaneId("l:west_right_turn_segment_0")},
      {LaneId("l:north_right_turn_segment_0")}, {LaneId("l:south_right_turn_segment_0")},
      {LaneId("l:east_left_turn_segment_0")},   {LaneId("l:west_left_turn_segment_0")},
      {LaneId("l:north_left_turn_segment_0")},  {LaneId("l:south_left_turn_segment_0")},
  };
  for (const auto& test_case : inside_intersection_test_cases) {
    const Lane* lane = dut_->ById().GetLane(test_case);
    EXPECT_NE(lane, nullptr);
    EXPECT_EQ(lane->GetOngoingBranches(LaneEnd::kStart)->size(), 1);
    EXPECT_EQ(lane->GetOngoingBranches(LaneEnd::kFinish)->size(), 1);
    EXPECT_EQ(lane->GetConfluentBranches(LaneEnd::kStart)->size(), 3);
    EXPECT_EQ(lane->GetConfluentBranches(LaneEnd::kFinish)->size(), 3);
  }
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
