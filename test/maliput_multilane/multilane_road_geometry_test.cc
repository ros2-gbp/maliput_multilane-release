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
/* clang-format off to disable clang-format-includes */
#include "maliput_multilane/road_geometry.h"
/* clang-format on */

#include <cmath>
#include <map>
#include <tuple>
#include <utility>

#include <gtest/gtest.h>
#include <maliput/common/maliput_abort.h>
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_multilane/builder.h"

namespace maliput {
namespace multilane {
namespace {

using api::HBounds;
using api::RBounds;
using multilane::ArcOffset;
using Which = api::LaneEnd::Which;

const double kVeryExact{1e-11};
const double kWidth{2.};   // Half lane width, used to feed the BuilderFactory.
const double kHeight{5.};  // Elevation bound.

const api::Lane* GetLaneByJunctionId(const api::RoadGeometry& rg, const std::string& junction_id, int segment_index,
                                     int lane_index) {
  MALIPUT_DEMAND(segment_index >= 0);
  MALIPUT_DEMAND(lane_index >= 0);

  for (int i = 0; i < rg.num_junctions(); ++i) {
    if (rg.junction(i)->id() == api::JunctionId(junction_id)) {
      if (segment_index >= rg.junction(i)->num_segments()) {
        throw std::runtime_error("Segment index is greater than available segment number.");
      }
      if (lane_index >= rg.junction(i)->segment(segment_index)->num_lanes()) {
        throw std::runtime_error("Lane index is greater than available lane number.");
      }
      return rg.junction(i)->segment(segment_index)->lane(lane_index);
    }
  }
  throw std::runtime_error("No matching junction name in the road network");
}

const api::Lane* GetLaneByJunctionId(const api::RoadGeometry& rg, const std::string& junction_id) {
  return GetLaneByJunctionId(rg, junction_id, 0, 0);
}

class MultilaneLanesQueriesTest : public ::testing::Test {
 public:
  void SetUp() override {
    const Connection* lane0 =
        rb_->Connect("lane0", kMonolaneLayout, StartReference().at(kRoadOrigin, Direction::kForward),
                     ArcOffset(kArcRadius, -kArcDeltaTheta), EndReference().z_at(kFlatZ, Direction::kForward));

    const Connection* lane1 =
        rb_->Connect("lane1", kMonolaneLayout, StartReference().at(*lane0, Which::kFinish, Direction::kForward),
                     LineOffset(kLength), EndReference().z_at(kFlatZ, Direction::kForward));

    const Connection* lane2 =
        rb_->Connect("lane2", kMonolaneLayout, StartReference().at(*lane1, Which::kFinish, Direction::kForward),
                     ArcOffset(kArcRadius, kArcDeltaTheta), EndReference().z_at(kFlatZ, Direction::kForward));
    rb_->Connect("lane3a", kMonolaneLayout, StartReference().at(*lane2, Which::kFinish, Direction::kForward),
                 LineOffset(kLength), EndReference().z_at(kFlatZ, Direction::kForward));

    rb_->Connect("lane3b", kMonolaneLayout, StartReference().at(*lane2, Which::kFinish, Direction::kForward),
                 ArcOffset(kArcRadius, kArcDeltaTheta), EndReference().z_at(kFlatZ, Direction::kForward));

    rb_->Connect("lane3c", kMonolaneLayout, StartReference().at(*lane2, Which::kFinish, Direction::kForward),
                 ArcOffset(kArcRadius, -kArcDeltaTheta), EndReference().z_at(kFlatZ, Direction::kForward));
    rg_ = rb_->Build(api::RoadGeometryId{"multi_lane_with_branches"});
    ASSERT_NE(rg_, nullptr);
  }

  // Define a serpentine road with multiple segments and branches.
  static constexpr double kLinearTolerance = 0.01;
  static constexpr double kAngularTolerance = 0.01 * M_PI;
  static constexpr double kScaleLength = 1.0;
  static const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};
  static constexpr double kArcDeltaTheta{M_PI / 2.};
  static constexpr double kArcRadius{50.};
  static constexpr double kLength{50.};
  static constexpr int kOneLane{1};
  static constexpr double kZeroR0{0.};
  static constexpr double kNoShoulder{0.};
  static constexpr int kRefLane{0};
  // Initialize the road from the origin.
  const multilane::EndpointXy kOriginXy{0., 0., 0.};
  const multilane::EndpointZ kFlatZ{0., 0., 0., 0.};
  const multilane::Endpoint kRoadOrigin{kOriginXy, kFlatZ};

  const LaneLayout kMonolaneLayout{kNoShoulder, kNoShoulder, kOneLane, kRefLane, kZeroR0};

  std::unique_ptr<BuilderBase> rb_ = multilane::BuilderFactory().Make(
      2. * kWidth, HBounds(0., kHeight), kLinearTolerance, kAngularTolerance, kScaleLength, kComputationPolicy);
  std::unique_ptr<const api::RoadGeometry> rg_;
};

TEST_F(MultilaneLanesQueriesTest, DoToRoadPosition) {
  // Place a point at the middle of lane1.
  api::InertialPosition inertial_pos{kArcRadius, -kArcRadius - kLength / 2., 0.};

  api::InertialPosition nearest_position{};
  api::RoadPositionResult result = rg_->ToRoadPosition(inertial_pos);

  // Expect to locate the point centered within lane1 (straight segment).
  EXPECT_TRUE(api::test::IsLanePositionClose(
      result.road_position.pos, api::LanePosition(kLength / 2. /* s */, 0. /* r */, 0. /* h */), kVeryExact));
  EXPECT_EQ(result.road_position.lane->id(), api::LaneId("l:lane1_0"));
  EXPECT_EQ(result.distance, 0.);
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position, api::InertialPosition(inertial_pos.x(), inertial_pos.y(), inertial_pos.z()),
      kVeryExact));

  // Place a point halfway to the end of lane1, just to the outside (left side)
  // of the lane bounds.
  inertial_pos = api::InertialPosition(kArcRadius + 2. * kWidth, -kArcRadius - kLength / 2., 0.);
  result = rg_->ToRoadPosition(inertial_pos);

  // Expect to locate the point just outside (to the left) of lane1, by an
  // amount kWidth.
  EXPECT_TRUE(api::test::IsLanePositionClose(
      result.road_position.pos, api::LanePosition(kLength / 2. /* s */, kWidth /* r */, 0. /* h */), kVeryExact));
  EXPECT_EQ(result.road_position.lane->id(), api::LaneId("l:lane1_0"));
  EXPECT_EQ(result.distance, kWidth);
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position, api::InertialPosition(inertial_pos.x() - kWidth, inertial_pos.y(), inertial_pos.z()),
      kVeryExact));

  // Place a point at the middle of lane3a (straight segment).
  inertial_pos = api::InertialPosition(2. * kArcRadius + kLength / 2., -2. * kArcRadius - kLength, 0.);
  result = rg_->ToRoadPosition(inertial_pos);

  // Expect to locate the point centered within lane3a.
  EXPECT_TRUE(api::test::IsLanePositionClose(
      result.road_position.pos, api::LanePosition(kLength / 2. /* s */, 0. /* r */, 0. /* h */), kVeryExact));
  EXPECT_EQ(result.road_position.lane->id(), api::LaneId("l:lane3a_0"));
  EXPECT_EQ(result.distance, 0.);
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position, api::InertialPosition(inertial_pos.x(), inertial_pos.y(), inertial_pos.z()),
      kVeryExact));

  // Place a point high above the middle of lane3a (straight segment).
  inertial_pos = api::InertialPosition(2. * kArcRadius + kLength / 2., -2. * kArcRadius - kLength, 50.);
  result = rg_->ToRoadPosition(inertial_pos);

  // Expect to locate the point centered above lane3a.
  EXPECT_TRUE(api::test::IsLanePositionClose(
      result.road_position.pos, api::LanePosition(kLength / 2. /* s */, 0. /* r */, kHeight /* h */), kVeryExact));
  EXPECT_EQ(result.road_position.lane->id(), api::LaneId("l:lane3a_0"));
  EXPECT_EQ(result.distance, 50. - kHeight);
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position, api::InertialPosition(inertial_pos.x(), inertial_pos.y(), kHeight), kVeryExact));

  // Place a point at the end of lane3b (arc segment).
  inertial_pos = api::InertialPosition(2. * kArcRadius + kLength, -kArcRadius - kLength, 0.);
  result = rg_->ToRoadPosition(inertial_pos);

  // Expect to locate the point at the end of lane3b.
  EXPECT_TRUE(api::test::IsLanePositionClose(
      result.road_position.pos, api::LanePosition(kArcRadius * M_PI / 2. /* s */, 0. /* r */, 0. /* h */), kVeryExact));
  EXPECT_EQ(result.road_position.lane->id(), api::LaneId("l:lane3b_0"));
  EXPECT_EQ(result.distance, 0.);
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position, api::InertialPosition(inertial_pos.x(), inertial_pos.y(), inertial_pos.z()),
      kVeryExact));

  // Supply a hint with a position at the start of lane3c to try and determine
  // the RoadPosition for a point at the end of lane3b.
  api::RoadPosition hint{GetLaneByJunctionId(*rg_, "j:lane3c"), {0., 0., 0.}};
  result = rg_->ToRoadPosition(inertial_pos, hint);

  // Expect to locate the point outside of lanes lane3c (and ongoing adjacent
  // lanes), since lane3b is not ongoing from lane3c.
  EXPECT_EQ(result.road_position.lane->id(), api::LaneId("l:lane3c_0"));
  EXPECT_GT(result.distance, 0.);  // inertial_pos is not within this lane.

  // Supply a hint with a position at the start of lane2 to try and determine
  // the RoadPosition for a point at the end of lane3b.
  hint = api::RoadPosition{GetLaneByJunctionId(*rg_, "j:lane2"), {0., 0., 0.}};
  result = rg_->ToRoadPosition(inertial_pos, hint);

  // Expect to traverse to lane3b (an ongoing lane) and then locate the point
  // within lane lane3b.
  EXPECT_EQ(result.road_position.lane->id(), api::LaneId("l:lane3b_0"));
  EXPECT_EQ(result.distance, 0.);  // inertial_pos is inside lane3b.
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position, api::InertialPosition(inertial_pos.x(), inertial_pos.y(), inertial_pos.z()),
      kVeryExact));
}

TEST_F(MultilaneLanesQueriesTest, DoToFindRoadPositions) {
  // Place a point at the middle of lane1 with a radius that should find only the lane1.
  api::InertialPosition inertial_pos{kArcRadius, -kArcRadius - kLength / 2., 0.};
  double radius = 0.5;
  api::InertialPosition nearest_position{};

  std::vector<api::RoadPositionResult> results = rg_->FindRoadPositions(inertial_pos, radius);
  ASSERT_EQ(static_cast<int>(results.size()), 1);
  api::RoadPositionResult result = results[0];

  // Expect to locate the point centered within lane1 (straight segment).
  EXPECT_TRUE(api::test::IsLanePositionClose(
      result.road_position.pos, api::LanePosition(kLength / 2. /* s */, 0. /* r */, 0. /* h */), kVeryExact));
  EXPECT_EQ(result.road_position.lane->id(), api::LaneId("l:lane1_0"));
  EXPECT_EQ(result.distance, 0.);
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position, api::InertialPosition(inertial_pos.x(), inertial_pos.y(), inertial_pos.z()),
      kVeryExact));

  // Place the point at the end of lane1. Lane1 and lane2 should be found.
  inertial_pos = {kArcRadius, -kArcRadius - kLength, 0.};
  results = rg_->FindRoadPositions(inertial_pos, radius);
  ASSERT_EQ(static_cast<int>(results.size()), 2);

  auto find_lane_in_results = [](const api::LaneId& lane_id, const std::vector<api::RoadPositionResult>& results) {
    auto lane_itr = std::find_if(results.begin(), results.end(), [&lane_id](const api::RoadPositionResult& result) {
      return result.road_position.lane->id() == lane_id;
    });
    return lane_itr;
  };

  // Checking l:lane1_0 result.
  auto lane_1_0_itr = find_lane_in_results(api::LaneId("l:lane1_0"), results);
  ASSERT_NE(lane_1_0_itr, results.end());
  result = *lane_1_0_itr;

  // Expect to locate the point at the end of lane1.
  const api::InertialPosition inertial_pos_lane_1{kArcRadius, -kArcRadius - kLength, 0.};
  EXPECT_TRUE(api::test::IsLanePositionClose(result.road_position.pos,
                                             api::LanePosition(kLength /* s */, 0. /* r */, 0. /* h */), kVeryExact));
  EXPECT_EQ(result.road_position.lane->id(), api::LaneId("l:lane1_0"));
  EXPECT_EQ(result.distance, 0.);
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position,
      api::InertialPosition(inertial_pos_lane_1.x(), inertial_pos_lane_1.y(), inertial_pos_lane_1.z()), kVeryExact));

  // Checking l:lane2_0 result.
  auto lane_2_0_itr = find_lane_in_results(api::LaneId("l:lane2_0"), results);
  ASSERT_NE(lane_2_0_itr, results.end());
  result = *lane_2_0_itr;

  // Expect to locate the point at the end of lane1.
  const api::InertialPosition inertial_pos_lane_2{kArcRadius, -kArcRadius - kLength, 0.};
  EXPECT_TRUE(api::test::IsLanePositionClose(result.road_position.pos,
                                             api::LanePosition(0. /* s */, 0. /* r */, 0. /* h */), kVeryExact));
  EXPECT_EQ(result.road_position.lane->id(), api::LaneId("l:lane2_0"));
  EXPECT_EQ(result.distance, 0.);
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position,
      api::InertialPosition(inertial_pos_lane_2.x(), inertial_pos_lane_2.y(), inertial_pos_lane_2.z()), kVeryExact));
}

GTEST_TEST(MultilaneLanesTest, HintWithDisconnectedLanes) {
  // Define a road with two disconnected, diverging lanes such that there are no
  // ongoing lanes.  This tests the pathological case when a `hint` is provided
  // in a topologically isolated lane, so the code returns the default road
  // position given by the hint.
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  const double kScaleLength = 1.0;
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};
  auto rb = multilane::BuilderFactory().Make(2. * kWidth, HBounds(0., kHeight), kLinearTolerance, kAngularTolerance,
                                             kScaleLength, kComputationPolicy);

  // Initialize the road from the origin.
  const multilane::EndpointXy kOriginXy0{0., 0., 0.};
  const multilane::EndpointXy kOriginXy1{0., 100., 0.};
  const multilane::EndpointZ kFlatZ{0., 0., 0., 0.};
  const multilane::Endpoint kRoadOrigin0{kOriginXy0, kFlatZ};
  const multilane::Endpoint kRoadOrigin1{kOriginXy1, kFlatZ};
  const int kOneLane{1};
  const int kRefLane{0};
  const double kZeroR0{0.};
  const double kNoShoulder{0.};

  // Define the lanes and connections.
  const LaneLayout kMonolaneLayout(kNoShoulder, kNoShoulder, kOneLane, kRefLane, kZeroR0);

  rb->Connect("lane0", kMonolaneLayout, StartReference().at(kRoadOrigin0, Direction::kForward),
              ArcOffset(50., -M_PI / 2.), EndReference().z_at(kFlatZ, Direction::kForward));

  rb->Connect("lane1", kMonolaneLayout, StartReference().at(kRoadOrigin1, Direction::kForward),
              ArcOffset(50., M_PI / 2.), EndReference().z_at(kFlatZ, Direction::kForward));

  std::unique_ptr<const api::RoadGeometry> rg = rb->Build(api::RoadGeometryId{"disconnected_lanes"});

  // Place a point at the middle of lane0.
  api::InertialPosition inertial_pos{50. * std::sqrt(2.) / 2., -50. * std::sqrt(2.) / 2., 0.};

  // Supply a hint with a position at the start of lane1.
  const api::RoadPosition hint{GetLaneByJunctionId(*rg, "j:lane1"), {0., 0., 0.}};
  api::RoadPositionResult result;
  EXPECT_NO_THROW(result = rg->ToRoadPosition(inertial_pos, hint));
  // The search is confined to lane1.
  EXPECT_EQ(result.road_position.lane->id(), api::LaneId("l:lane1_0"));
  // lane1 does not contain the point.
  EXPECT_GT(result.distance, 0.);
}

// Tests different api::Geoposition in the following RoadGeometry without a
// hint.
//
//        ^ +r, +y
//        |
//        |                 g
//        ------------------------------------
//        |                 f                |           Left shoulder
//        ------------------------------------
//        |                 e                |           l:2
//        |                                  |
//        ------------------------------------
//        |                                  |           l:1
//        |                 d                |
//        ------------------c-----------------
// (0,0,0)|__________________________________|_____> +s  l:0
//        |                                  |       +x
//        ------------------------------------
//        |                 b                |           Right shoulder
//        ------------------------------------
//                          a
//
// Letters, such as `a`, `b`, etc. are the api::InertialPositions to test.
GTEST_TEST(MultilaneLanesTest, MultipleLineLaneSegmentWithoutHint) {
  const HBounds kElevationBounds{0., kHeight};
  const double kLinearTolerance{kVeryExact};
  const double kAngularTolerance{0.01 * M_PI};
  const double kLaneWidth{2. * kWidth};
  const double kScaleLength = 1.0;
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};
  auto builder = multilane::BuilderFactory().Make(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
                                                  kScaleLength, kComputationPolicy);

  // Initialize the road from the origin.
  const EndpointZ kFlatZ{0., 0., 0., 0.};
  const double kZeroZ{0.};
  const Endpoint kRoadOrigin{{0., 0., 0.}, kFlatZ};
  const double kLength{10.};
  const double kHalfLength{0.5 * kLength};
  const int kThreeLanes{3};
  const int kRefLane{0};
  const double kZeroR0{0.};
  const double kShoulder{1.0};
  const LaneLayout kThreeLaneLayout(kShoulder, kShoulder, kThreeLanes, kRefLane, kZeroR0);
  // Creates a simple 3-line-lane segment road.
  builder->Connect("s0", kThreeLaneLayout, StartReference().at(kRoadOrigin, Direction::kForward), LineOffset(kLength),
                   EndReference().z_at(kFlatZ, Direction::kForward));
  std::unique_ptr<const api::RoadGeometry> rg = builder->Build(api::RoadGeometryId{"multi-lane-line-segment"});

  // Prepares the truth table to match different api::InertialPositions into
  // api::RoadPositions.
  const api::Lane* kFirstLane = GetLaneByJunctionId(*rg, "j:s0", 0, 0);
  const api::Lane* kSecondLane = GetLaneByJunctionId(*rg, "j:s0", 0, 1);
  const api::Lane* kThirdLane = GetLaneByJunctionId(*rg, "j:s0", 0, 2);

  // <Geo point - Expected road pos - Nearest pos - Hint - Distance >
  const std::vector<
      std::tuple<api::InertialPosition, api::RoadPosition, api::InertialPosition, api::RoadPosition, double>>
      truth_vector{
          std::make_tuple<api::InertialPosition, api::RoadPosition, api::InertialPosition, api::RoadPosition,
                          double>(  // a
              {kHalfLength, -kLaneWidth, kZeroZ}, {kFirstLane, {kHalfLength, -0.75 * kLaneWidth, kZeroZ}},
              {kHalfLength, -0.75 * kLaneWidth, kZeroZ}, {kFirstLane, {}}, 0.25 * kLaneWidth),
          std::make_tuple<api::InertialPosition, api::RoadPosition, api::InertialPosition, api::RoadPosition,
                          double>(  // b
              {kHalfLength, -0.625 * kLaneWidth, kZeroZ}, {kFirstLane, {kHalfLength, -0.625 * kLaneWidth, kZeroZ}},
              {kHalfLength, -0.625 * kLaneWidth, kZeroZ}, {kFirstLane, {}}, 0.),
          std::make_tuple<api::InertialPosition, api::RoadPosition, api::InertialPosition, api::RoadPosition,
                          double>(  // c
              {kHalfLength, 0.5 * kLaneWidth, kZeroZ}, {kSecondLane, {kHalfLength, -0.5 * kLaneWidth, kZeroZ}},
              {kHalfLength, 0.5 * kLaneWidth, kZeroZ}, {kSecondLane, {}}, 0.),
          std::make_tuple<api::InertialPosition, api::RoadPosition, api::InertialPosition, api::RoadPosition,
                          double>(  // d
              {kHalfLength, 0.775 * kLaneWidth, kZeroZ}, {kSecondLane, {kHalfLength, -0.225 * kLaneWidth, kZeroZ}},
              {kHalfLength, 0.775 * kLaneWidth, kZeroZ}, {kSecondLane, {}}, 0.),
          std::make_tuple<api::InertialPosition, api::RoadPosition, api::InertialPosition, api::RoadPosition,
                          double>(  // e
              {kHalfLength, 2.075 * kLaneWidth, kZeroZ}, {kThirdLane, {kHalfLength, 0.075 * kLaneWidth, kZeroZ}},
              {kHalfLength, 2.075 * kLaneWidth, kZeroZ}, {kThirdLane, {}}, 0.),
          std::make_tuple<api::InertialPosition, api::RoadPosition, api::InertialPosition, api::RoadPosition,
                          double>(  // f
              {kHalfLength, 2.625 * kLaneWidth, kZeroZ}, {kThirdLane, {kHalfLength, 0.625 * kLaneWidth, kZeroZ}},
              {kHalfLength, 2.625 * kLaneWidth, kZeroZ}, {kThirdLane, {}}, 0.),
          std::make_tuple<api::InertialPosition, api::RoadPosition, api::InertialPosition, api::RoadPosition,
                          double>(  // g
              {kHalfLength, 3. * kLaneWidth, kZeroZ}, {kThirdLane, {kHalfLength, 0.75 * kLaneWidth, kZeroZ}},
              {kHalfLength, 2.75 * kLaneWidth, kZeroZ}, {kThirdLane, {}}, 0.25 * kLaneWidth),
      };

  api::RoadPositionResult result{};
  // Evaluates the truth table without a hint.
  for (const auto truth_value : truth_vector) {
    EXPECT_NO_THROW(result = rg->ToRoadPosition(std::get<0>(truth_value)));
    EXPECT_EQ(result.road_position.lane->id(), std::get<1>(truth_value).lane->id());
    EXPECT_TRUE(
        api::test::IsLanePositionClose(result.road_position.pos, std::get<1>(truth_value).pos, kLinearTolerance));
    EXPECT_TRUE(
        api::test::IsInertialPositionClose(result.nearest_position, std::get<2>(truth_value), kLinearTolerance));
    EXPECT_NEAR(result.distance, std::get<4>(truth_value), kLinearTolerance);
  }

  // Evaluates the truth table with a hint.
  for (const auto truth_value : truth_vector) {
    EXPECT_NO_THROW(result = rg->ToRoadPosition(std::get<0>(truth_value), std::get<3>(truth_value)));
    EXPECT_EQ(result.road_position.lane->id(), std::get<1>(truth_value).lane->id());
    EXPECT_TRUE(
        api::test::IsLanePositionClose(result.road_position.pos, std::get<1>(truth_value).pos, kLinearTolerance));
    EXPECT_TRUE(
        api::test::IsInertialPositionClose(result.nearest_position, std::get<2>(truth_value), kLinearTolerance));
    EXPECT_NEAR(result.distance, std::get<4>(truth_value), kLinearTolerance);
  }
}

// This test creates a line segment that overlaps with an arc segment. Both
// segments are single-lane segments whose lane curves start at the same point.
// This tests exercises RoadGeometry::ToRoadPosition() to verify that the
// correct positions are obtained with overlapped lane bounds.
//
// <pre>
//                     - arc-segment
//                   --
//                 --
//              ---
//           ---   c
//       ----
//   ----          b
//  a------------------------------- line-segment
//
// </pre>
//
// `a`, `b` and `c` are the InertialPositions to test.
GTEST_TEST(MultilaneLanesTest, OverlappingLaneBounds) {
  const HBounds kElevationBounds{0., kHeight};
  const double kLinearTolerance{kVeryExact};   // meters.
  const double kAngularTolerance{kVeryExact};  // radians.
  const double kLaneWidth{2. * kWidth};
  const double kScaleLength = 1.0;
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};
  auto builder = multilane::BuilderFactory().Make(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance,
                                                  kScaleLength, kComputationPolicy);

  // Initialize the road from the origin.
  const EndpointZ kFlatZ{0., 0., 0., 0.};
  const Endpoint kRoadOrigin{{0., 0., 0.}, kFlatZ};
  const double kLength{50.};
  const double kRadius{50.};
  const double kDTheta{M_PI / 4.};
  const int kOneLane{1};
  const int kRefLane{0};
  const double kZeroR0{0.};
  const double kShoulder{2.0};
  const LaneLayout kMonolaneLayout(kShoulder, kShoulder, kOneLane, kRefLane, kZeroR0);

  // Creates a simple straight-line segment road.
  builder->Connect("line-segment", kMonolaneLayout, StartReference().at(kRoadOrigin, Direction::kForward),
                   LineOffset(kLength), EndReference().z_at(kFlatZ, Direction::kForward));
  // Creates a simple curve segment road.
  builder->Connect("arc-segment", kMonolaneLayout, StartReference().at(kRoadOrigin, Direction::kForward),
                   ArcOffset(kRadius, kDTheta), EndReference().z_at(kFlatZ, Direction::kForward));

  std::unique_ptr<const api::RoadGeometry> rg = builder->Build(api::RoadGeometryId{"multi-lane-line-segment"});

  // Prepares the truth table to match different api::InertialPositions into
  // api::RoadPositions.
  const api::Lane* line_lane = GetLaneByJunctionId(*rg, "j:line-segment", 0, 0);
  const api::Lane* arc_lane = GetLaneByJunctionId(*rg, "j:arc-segment", 0, 0);

  api::RoadPositionResult result{};

  // Checks the origin, where both lanes start from the same InertialPosition and
  // there is no explicit preference for one or the other lane in maliput API
  // as both have the same `r` coordinate and no hint is supplied.
  const api::InertialPosition kAPoint(0., 0., 0.);
  EXPECT_NO_THROW(result = rg->ToRoadPosition(kAPoint));
  EXPECT_NE(result.road_position.lane, nullptr);
  EXPECT_TRUE(
      api::test::IsLanePositionClose(result.road_position.pos, api::LanePosition(0., 0., 0.), kLinearTolerance));
  EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, kAPoint, kLinearTolerance));
  EXPECT_NEAR(result.distance, 0., kLinearTolerance);

  // Using the same InertialPosition, but with a line-hint, we should get the
  // result on the line-segment.
  const api::RoadPosition kStartLineLane{line_lane, api::LanePosition(0., 0., 0.)};
  EXPECT_NO_THROW(result = rg->ToRoadPosition(kAPoint, kStartLineLane));
  EXPECT_EQ(result.road_position.lane->id(), line_lane->id());
  EXPECT_TRUE(
      api::test::IsLanePositionClose(result.road_position.pos, api::LanePosition(0., 0., 0.), kLinearTolerance));
  EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, kAPoint, kLinearTolerance));
  EXPECT_NEAR(result.distance, 0., kLinearTolerance);

  // Same as before, but with a hint on the arc-segment.
  const api::RoadPosition kStartArcLane{arc_lane, api::LanePosition(0., 0., 0.)};
  EXPECT_NO_THROW(result = rg->ToRoadPosition(kAPoint, kStartArcLane));
  EXPECT_EQ(result.road_position.lane->id(), arc_lane->id());
  EXPECT_TRUE(
      api::test::IsLanePositionClose(result.road_position.pos, api::LanePosition(0., 0., 0.), kLinearTolerance));
  EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, kAPoint, kLinearTolerance));
  EXPECT_NEAR(result.distance, 0., kLinearTolerance);

  // Tests a InertialPosition in between line and arc lane curves. It is closer to
  // the line segment (smaller `r` coordinate) but within the lane bounds of
  // both road lanes.
  // The following constants explain how to locate the point in space. An
  // arbitrary kROffset distance from the curve-centerline was chosen as well
  // as an `x` coordinate. The rest of the variables are derived from them.
  const double kX{8.68};
  const double kROffset{0.3};
  const api::InertialPosition kBPoint(kX, kROffset, 0.);
  EXPECT_NO_THROW(result = rg->ToRoadPosition(kBPoint));
  EXPECT_EQ(result.road_position.lane->id(), line_lane->id());
  EXPECT_TRUE(
      api::test::IsLanePositionClose(result.road_position.pos, api::LanePosition(kX, kROffset, 0.), kLinearTolerance));
  EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, kBPoint, kLinearTolerance));
  EXPECT_NEAR(result.distance, 0., kLinearTolerance);

  // Tests a InertialPosition in between line and arc lane curves. It is closer to
  // the arc segment (smaller `r` coordinate) but within the lane bounds of
  // both road lanes.
  // Using previous constants, another point is derived but closer to the
  // arc-centerline.
  const double kCTheta{std::asin(kX / (kRadius + kROffset))};
  const double kCY{kRadius - (kRadius + kROffset) * std::cos(kCTheta)};
  const api::InertialPosition kCPoint(kX, kCY, 0.);
  EXPECT_NO_THROW(result = rg->ToRoadPosition(kCPoint));
  EXPECT_EQ(result.road_position.lane->id(), arc_lane->id());
  EXPECT_TRUE(api::test::IsLanePositionClose(result.road_position.pos,
                                             api::LanePosition(kRadius * kCTheta, -kROffset, 0.), kLinearTolerance));
  EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, kCPoint, kLinearTolerance));
  EXPECT_NEAR(result.distance, 0., kLinearTolerance);
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
