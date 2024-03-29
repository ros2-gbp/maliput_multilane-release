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
#include "maliput_multilane/lane.h"
/* clang-format on */

#include <cmath>
#include <map>

#include <gtest/gtest.h>
#include <maliput/test_utilities/maliput_math_compare.h>
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_multilane/arc_road_curve.h"
#include "maliput_multilane/junction.h"
#include "maliput_multilane/line_road_curve.h"
#include "maliput_multilane/road_curve.h"
#include "maliput_multilane/road_geometry.h"
#include "maliput_multilane/segment.h"

namespace maliput {
namespace multilane {
namespace {

using maliput::math::test::CompareVectors;

const double kLinearTolerance = 1e-6;
const double kAngularTolerance = 1e-6;
const double kVeryExact = 1e-12;

GTEST_TEST(MultilaneLanesTest, Rot3) {
  // Spot-check that Rot3 is behaving as advertised.
  Rot3 rpy90{M_PI / 2., M_PI / 2., M_PI / 2.};
  EXPECT_TRUE(CompareVectors(rpy90.apply({1., 0., 0.}), math::Vector3(0., 0., -1.), kVeryExact));
  EXPECT_TRUE(CompareVectors(rpy90.apply({0., 1., 0.}), math::Vector3(0., 1., 0.), kVeryExact));
  EXPECT_TRUE(CompareVectors(rpy90.apply({0., 0., 1.}), math::Vector3(1., 0., 0.), kVeryExact));
}

class MultilaneLanesParamTest : public ::testing::TestWithParam<double> {
 protected:
  void SetUp() override { r0 = this->GetParam(); }

  const double kScaleLength{1.};
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};
  const CubicPolynomial zp{0., 0., 0., 0.};
  const double kHalfWidth{10.};
  const double kMaxHeight{5.};
  const double kHalfLaneWidth{5.};
  double r0{};
};

TEST_P(MultilaneLanesParamTest, FlatLineLane) {
  RoadGeometry rg(api::RoadGeometryId{"apple"}, kLinearTolerance, kAngularTolerance, kScaleLength);
  std::unique_ptr<RoadCurve> road_curve_1 = std::make_unique<LineRoadCurve>(
      math::Vector2(100., -75.), math::Vector2(100., 50.), zp, zp, kLinearTolerance, kScaleLength, kComputationPolicy);
  const math::Vector3 s_vector = math::Vector3(100., 50., 0.).normalized();
  const math::Vector3 r_vector = math::Vector3(-50, 100., 0.).normalized();
  const math::Vector3 r_offset_vector = r0 * r_vector;
  Segment* s1 = rg.NewJunction(api::JunctionId{"j1"})
                    ->NewSegment(api::SegmentId{"s1"}, std::move(road_curve_1), -kHalfWidth + r0, kHalfWidth + r0,
                                 {0., kMaxHeight});
  Lane* l1 = s1->NewLane(api::LaneId{"l1"}, r0, {-kHalfLaneWidth, kHalfLaneWidth});

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_EQ(l1->id(), api::LaneId("l1"));
  EXPECT_EQ(l1->segment(), s1);
  EXPECT_EQ(l1->index(), 0);
  EXPECT_EQ(l1->to_left(), nullptr);
  EXPECT_EQ(l1->to_right(), nullptr);
  EXPECT_EQ(l1->r0(), r0);

  EXPECT_NEAR(l1->length(), std::sqrt((100. * 100) + (50. * 50.)), kVeryExact);

  EXPECT_TRUE(
      api::test::IsRBoundsClose(l1->lane_bounds(0.), api::RBounds(-kHalfLaneWidth, kHalfLaneWidth), kVeryExact));
  EXPECT_TRUE(api::test::IsRBoundsClose(l1->segment_bounds(0.), api::RBounds(-kHalfWidth, kHalfWidth), kVeryExact));
  EXPECT_TRUE(api::test::IsHBoundsClose(l1->elevation_bounds(0., 0.), api::HBounds(0., kMaxHeight), kVeryExact));

  EXPECT_TRUE(api::test::IsInertialPositionClose(
      l1->ToInertialPosition({0., 0., 0.}),
      api::InertialPosition::FromXyz(math::Vector3(100., -75., 0.) + r_offset_vector), kLinearTolerance));

  // A little bit along the lane, but still on the reference line.
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      l1->ToInertialPosition({1., 0., 0.}),
      api::InertialPosition::FromXyz(
          math::Vector3(100. + ((100. / l1->length()) * 1.), -75. + ((50. / l1->length()) * 1.), 0.) + r_offset_vector),
      kLinearTolerance));
  // At the very beginning of the lane, but laterally off the reference line.
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      l1->ToInertialPosition({0., 3., 0.}),
      api::InertialPosition::FromXyz(
          math::Vector3(100. + ((-50. / l1->length()) * 3.), -75. + ((100. / l1->length()) * 3.), 0.) +
          r_offset_vector),
      kLinearTolerance));
  // At the very end of the lane.
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      l1->ToInertialPosition({l1->length(), 0., 0.}),
      api::InertialPosition(200. + r_offset_vector.x(), -25. + r_offset_vector.y(), 0. + r_offset_vector.z()),
      kLinearTolerance));
  // Case 1: Tests LineLane::ToSegmentPosition() and LineLane::ToLanePosition() with a closest point that lies
  // within the lane bounds.
  const api::InertialPosition point_within_lane{150., -50., 0.};
  const math::Vector3 d_point_lane_origin = point_within_lane.xyz() - math::Vector3(100., -75., 0.) - r_offset_vector;
  const double expected_s = d_point_lane_origin.dot(s_vector);
  const double expected_r = d_point_lane_origin.dot(r_vector);

  // ToSegmentPosition
  api::LanePositionResult result = l1->ToSegmentPosition(point_within_lane);
  EXPECT_TRUE(
      api::test::IsLanePositionClose(result.lane_position, api::LanePosition(expected_s, expected_r, 0.), kVeryExact));
  EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, api::InertialPosition(150., -50., 0.),
                                                 kLinearTolerance));
  EXPECT_NEAR(result.distance, 0., kVeryExact);

  // ToLanePosition
  result = l1->ToLanePosition(point_within_lane);
  EXPECT_TRUE(
      api::test::IsLanePositionClose(result.lane_position, api::LanePosition(expected_s, expected_r, 0.), kVeryExact));
  EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, api::InertialPosition(150., -50., 0.),
                                                 kLinearTolerance));
  EXPECT_NEAR(result.distance, 0., kVeryExact);

  // Case 2: Tests LineLane::ToSegmentPosition() with a closest point that lies
  // outside of the segment bounds, verifying that the result saturates.
  const api::InertialPosition point_outside_lane{-75., 25., 20.};
  const double expected_r_outside_segment = kHalfWidth;

  // ToSegmentPosition
  result = l1->ToSegmentPosition(point_outside_lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(
      result.lane_position, api::LanePosition(0., expected_r_outside_segment, kMaxHeight), kVeryExact));
  const math::Vector3 extreme_segment_point = math::Vector3(100., -75, 0.0) + r_offset_vector +
                                              kHalfWidth * r_vector.normalized() + math::Vector3(0., 0., kMaxHeight);
  EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position,
                                                 api::InertialPosition::FromXyz(extreme_segment_point), kVeryExact));
  EXPECT_NEAR(result.distance, (point_outside_lane.xyz() - extreme_segment_point).norm(), kVeryExact);

  // ToLanePosition
  const double expected_r_outside_lane = kHalfLaneWidth;
  result = l1->ToLanePosition(point_outside_lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(result.lane_position,
                                             api::LanePosition(0., expected_r_outside_lane, kMaxHeight), kVeryExact));
  const math::Vector3 extreme_lane_point = math::Vector3(100., -75, 0.0) + r_offset_vector +
                                           kHalfLaneWidth * r_vector.normalized() + math::Vector3(0., 0., kMaxHeight);
  EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position,
                                                 api::InertialPosition::FromXyz(extreme_lane_point), kVeryExact));
  EXPECT_NEAR(result.distance, (point_outside_lane.xyz() - extreme_lane_point).norm(), kVeryExact);

  // Case 3: Tests LineLane::ToSegmentPosition() at a non-zero but flat elevation.
  const double elevation = 10.;
  const double length = std::sqrt(std::pow(100, 2.) + std::pow(50, 2.));
  std::unique_ptr<RoadCurve> road_curve_2 = std::make_unique<LineRoadCurve>(
      math::Vector2(100., -75.), math::Vector2(100., 50.), CubicPolynomial(elevation / length, 0.0, 0.0, 0.0), zp,
      kLinearTolerance, kScaleLength, kComputationPolicy);
  Segment* s2 = rg.NewJunction(api::JunctionId{"j2"})
                    ->NewSegment(api::SegmentId{"s2"}, std::move(road_curve_2), -kHalfWidth + r0, kHalfWidth + r0,
                                 {0., kMaxHeight});
  Lane* l1_with_z = s2->NewLane(api::LaneId{"l1_with_z"}, r0, {-kHalfLaneWidth, kHalfLaneWidth});

  result = l1_with_z->ToSegmentPosition(point_outside_lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(
      result.lane_position, api::LanePosition(0., expected_r_outside_segment, kMaxHeight), kVeryExact));
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position, api::InertialPosition::FromXyz(extreme_segment_point + math::Vector3(0., 0., elevation)),
      kVeryExact));
  EXPECT_NEAR(result.distance,
              (point_outside_lane.xyz() - extreme_segment_point - math::Vector3(0., 0., elevation)).norm(), kVeryExact);

  // Verifies the output of LineLane::GetOrientation().
  EXPECT_TRUE(api::test::IsRotationClose(l1->GetOrientation({0., 0., 0.}),
                                         api::Rotation::FromRpy(0., 0., std::atan2(50., 100.)), kVeryExact));

  EXPECT_TRUE(api::test::IsRotationClose(l1->GetOrientation({1., 0., 0.}),
                                         api::Rotation::FromRpy(0., 0., std::atan2(50., 100.)), kVeryExact));

  EXPECT_TRUE(api::test::IsRotationClose(l1->GetOrientation({0., 1., 0.}),
                                         api::Rotation::FromRpy(0., 0., std::atan2(50., 100.)), kVeryExact));

  EXPECT_TRUE(api::test::IsRotationClose(l1->GetOrientation({l1->length(), 0., 0.}),
                                         api::Rotation::FromRpy(0., 0., std::atan2(50., 100.)), kVeryExact));

  // Derivative map should be identity (for a flat, straight road).
  EXPECT_TRUE(api::test::IsLanePositionClose(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
                                             api::LanePosition(0., 0., 0.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l1->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
                                             api::LanePosition(1., 0., 0.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
                                             api::LanePosition(0., 1., 0.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l1->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
                                             api::LanePosition(0., 0., 1.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l1->EvalMotionDerivatives({0., 0., 0.}, {1., 1., 1.}),
                                             api::LanePosition(1., 1., 1.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l1->EvalMotionDerivatives({10., 5., 3.}, {1., 2., 3.}),
                                             api::LanePosition(1., 2., 3.), kVeryExact));
}

namespace {

// Exact corkscrew curve parameterization, for numerical
// approximation validation.
class CorkScrew {
 public:
  // Constructs a corkscrew with the given @p radius,
  // @p axial_length and @p number_of_turns.
  CorkScrew(const double& radius, const double& axial_length, const double& number_of_turns);

  // Returns the (x, y, z) position in the global frame at the
  // provided @p srh location on the corkscrew.
  math::Vector3 position_at_srh(const math::Vector3& srh) const;

  // Returns the (r, p, y) orientation triplet in the global frame
  // at the provided @p srh location on the corkscrew.
  math::Vector3 orientation_at_srh(const math::Vector3& srh) const;

  // Returns the (ṡ, ṙ, ḣ) velocity at the provided @p srh location
  // on the corkscrew, scaled by the @p iso_v velocity in the
  // (σ, ρ, η) frame i.e. a frame attached to the corkscrew frame
  // but isotropic with the global frame (such that said velocity
  // represents a real velocity).
  math::Vector3 motion_derivative_at_srh(const math::Vector3& srh, const math::Vector3& iso_v) const;

  // Returns the path length of the corkscrew.
  inline double length() const { return length_; }

 private:
  // The radius of the corkscrew.
  const double radius_;
  // The axial length of the corkscrew, as seen
  // if projected onto the x-axis, in meters.
  const double axial_length_;
  // The total angular rotation undergone by the
  // corkscrew, as seen if projected over the x = 0
  // plane, in rads.
  const double angular_length_;
  // The path length of the corkscrew, in meters.
  const double length_;
};

CorkScrew::CorkScrew(const double& radius, const double& axial_length, const double& number_of_turns)
    : radius_(radius),
      axial_length_(axial_length),
      angular_length_(2 * M_PI * number_of_turns),
      length_(std::sqrt(std::pow(axial_length_, 2) + std::pow(angular_length_ * radius_, 2))) {}

math::Vector3 CorkScrew::position_at_srh(const math::Vector3& srh) const {
  // TODO(hidmic): Assuming the LANE frame s-axis is always aligned
  //               with the GLOBAL frame x-axis is incorrect. However,
  //               this same bug is present in Multilane. Fix this
  //               computation when the implementation gets fixed.
  const double p = srh[0] / length();
  const double effective_r_offset = srh[1] + radius_;
  const double sgamma = std::sin(angular_length_ * p);
  const double cgamma = std::cos(angular_length_ * p);
  return math::Vector3(axial_length_ * p, effective_r_offset * cgamma - srh[2] * sgamma,
                       effective_r_offset * sgamma + srh[2] * cgamma);
}

math::Vector3 CorkScrew::motion_derivative_at_srh(const math::Vector3& srh, const math::Vector3& iso_v) const {
  // TODO(hidmic): Assuming the LANE frame s-axis is always aligned
  //               with the GLOBAL frame x-axis is incorrect. However,
  //               this same bug is present in Multilane. Fix this
  //               computation when the implementation gets fixed.
  const double p = srh[0] / length();
  const double sgamma = std::sin(angular_length_ * p);
  const double cgamma = std::cos(angular_length_ * p);
  const double alpha = angular_length_ * (srh[1] + radius_);
  const double alpha0 = angular_length_ * radius_;
  const double beta = angular_length_ * srh[2];
  const math::Vector3 position_derivative_at_p00(axial_length_, -alpha0 * sgamma, alpha0 * cgamma);
  const math::Vector3 position_derivative_at_prh(axial_length_, -alpha * sgamma - beta * cgamma,
                                                 alpha * cgamma - beta * sgamma);
  return math::Vector3(position_derivative_at_p00.norm() / position_derivative_at_prh.norm() * iso_v[0], iso_v[1],
                       iso_v[2]);
}

math::Vector3 CorkScrew::orientation_at_srh(const math::Vector3& srh) const {
  // TODO(hidmic): Assuming the LANE frame s-axis is always aligned
  //               with the GLOBAL frame x-axis is incorrect. However,
  //               this same bug is present in Multilane. Fix this
  //               computation when the implementation gets fixed.
  const double p = srh[0] / length();
  const double effective_r_offset = srh[1] + radius_;
  const double sgamma = std::sin(angular_length_ * p);
  const double cgamma = std::cos(angular_length_ * p);
  const math::Vector3 s_vec(axial_length_, -angular_length_ * (effective_r_offset * sgamma + srh[2] * cgamma),
                            angular_length_ * (effective_r_offset * cgamma - srh[2] * sgamma));
  const math::Vector3 s_hat = s_vec.normalized();
  const math::Vector3 r_hat(0., cgamma, sgamma);
  // TODO(hidmic): Make use of maliput::drake::math::RollPitchYaw:
  //
  // maliput::drake::Matrix3<T> rotmat;
  // rotmat << s_hat, r_hat, s_hat.cross(r_hat);
  // return maliput::drake::math::RollPitchYaw<T>(maliput::drake::math::RotationMatrix<T>(rotmat)).vector();
  //
  // Code below is a verbatim partial transcription of the
  // RoadCurve::Orientation() method implementation that, somehow, gives a
  // different output than that of above's code snippet for the same input.
  const double gamma = std::atan2(s_hat.y(), s_hat.x());
  const double beta = std::atan2(-s_hat.z(), math::Vector2(s_hat.x(), s_hat.y()).norm());
  const double cb = std::cos(beta);
  const double alpha = std::atan2(r_hat.z() / cb, ((r_hat.y() * s_hat.x()) - (r_hat.x() * s_hat.y())) / cb);
  return math::Vector3(alpha, beta, gamma);
}

}  // namespace

// Checks Lane position, orientation and motion derivative computations
// accuracy for a linearly superelevated, corkscrew-like baseline. The
// numerical based approach used by the RoadCurve to deal with this kind
// of complex shapes are compared against exact results.
TEST_P(MultilaneLanesParamTest, CorkScrewLane) {
  const int kTurns = 10;
  const double kLength = 20.;
  const CorkScrew corkscrew_curve(r0, kLength, kTurns);
  // Reproduces the same superelevation profile as that of the corkscrew.
  const CubicPolynomial corkscrew_polynomial(0., 2. * M_PI * kTurns / kLength, 0., 0.);

  // Road curve's scale length is computed as
  // half the path length of a single corkscrew
  // turn.
  const double kCorkscrewScaleLength = corkscrew_curve.length() / (2 * kTurns);
  std::unique_ptr<RoadCurve> road_curve =
      std::make_unique<LineRoadCurve>(math::Vector2(0., 0.), math::Vector2(kLength, 0.), zp, corkscrew_polynomial,
                                      kLinearTolerance, kCorkscrewScaleLength, kComputationPolicy);

  RoadGeometry rg(api::RoadGeometryId{"corkscrew"}, kLinearTolerance, kAngularTolerance, kScaleLength);
  Segment* s1 = rg.NewJunction(api::JunctionId{"j1"})
                    ->NewSegment(api::SegmentId{"s1"}, std::move(road_curve), -kHalfWidth + r0, kHalfWidth + r0,
                                 {0., kMaxHeight});
  Lane* l1 = s1->NewLane(api::LaneId{"l1"}, r0, {-kHalfLaneWidth, kHalfLaneWidth});

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_EQ(l1->id(), api::LaneId("l1"));
  EXPECT_EQ(l1->segment(), s1);
  EXPECT_EQ(l1->index(), 0);
  EXPECT_EQ(l1->to_left(), nullptr);
  EXPECT_EQ(l1->to_right(), nullptr);
  EXPECT_EQ(l1->r0(), r0);

  EXPECT_NEAR(l1->length(), corkscrew_curve.length(), kLinearTolerance);

  EXPECT_TRUE(
      api::test::IsRBoundsClose(l1->lane_bounds(0.), api::RBounds(-kHalfLaneWidth, kHalfLaneWidth), kVeryExact));
  EXPECT_TRUE(api::test::IsRBoundsClose(l1->segment_bounds(0.), api::RBounds(-kHalfWidth, kHalfWidth), kVeryExact));
  EXPECT_TRUE(api::test::IsHBoundsClose(l1->elevation_bounds(0., 0.), api::HBounds(0., kMaxHeight), kVeryExact));

  const api::IsoLaneVelocity lane_velocity(1., 10., 100.);
  const math::Vector3 lane_velocity_as_vector(lane_velocity.sigma_v, lane_velocity.rho_v, lane_velocity.eta_v);

  const std::vector<double> lane_position_s_offsets = {0., 1., l1->length() / 2., l1->length() - 1., l1->length()};
  const std::vector<double> lane_position_r_offsets = {-kHalfWidth, -kHalfWidth + 1., -1.,       0.,
                                                       1.,          kHalfWidth - 1.,  kHalfWidth};
  const std::vector<double> lane_position_h_offsets = {0., 1., kMaxHeight / 2., kMaxHeight - 1., kMaxHeight};

  for (double s_offset : lane_position_s_offsets) {
    for (double r_offset : lane_position_r_offsets) {
      for (double h_offset : lane_position_h_offsets) {
        // Instantiates lane position with current offsets.
        const api::LanePosition lane_position(s_offset, r_offset, h_offset);
        const auto lane_position_srh{lane_position.srh()};
        const auto position_at_srh_drake{
            corkscrew_curve.position_at_srh({lane_position_srh.x(), lane_position_srh.y(), lane_position_srh.z()})};
        const auto orientation_at_srh_drake{
            corkscrew_curve.orientation_at_srh({lane_position_srh.x(), lane_position_srh.y(), lane_position_srh.z()})};
        const auto motion_derivative_at_srh_drake{corkscrew_curve.motion_derivative_at_srh(
            {lane_position_srh.x(), lane_position_srh.y(), lane_position_srh.z()}, lane_velocity_as_vector)};
        // Checks position in the (x, y, z) frame i.e. world
        // down to kLinearTolerance accuracy (as that's the
        // tolerance the RoadGeometry was constructed with).
        EXPECT_TRUE(api::test::IsInertialPositionClose(
            l1->ToInertialPosition(lane_position),
            api::InertialPosition::FromXyz(
                {position_at_srh_drake.x(), position_at_srh_drake.y(), position_at_srh_drake.z()}),
            kLinearTolerance));

        // Checks orientation in the (x, y, z) frame i.e. world
        // down to kAngularTolerance accuracy (as that's the
        // tolerance the RoadGeometry was constructed with).
        EXPECT_TRUE(api::test::IsRotationClose(
            l1->GetOrientation(lane_position),
            api::Rotation::FromRpy(
                {orientation_at_srh_drake.x(), orientation_at_srh_drake.y(), orientation_at_srh_drake.z()}),
            kAngularTolerance));

        // Checks motion derivatives in the (s, r, h) frame i.e.
        // lane down to kLinearTolerance accuracy (as that's
        // the tolerance the RoadGeometry was constructed with).
        EXPECT_TRUE(api::test::IsLanePositionClose(
            l1->EvalMotionDerivatives(lane_position, lane_velocity),
            api::LanePosition::FromSrh({motion_derivative_at_srh_drake.x(), motion_derivative_at_srh_drake.y(),
                                        motion_derivative_at_srh_drake.z()}),
            kLinearTolerance));

        // TODO(hidmic): Add Lane::ToLanePosition() tests when the zero
        //               superelevation restriction in Multilane is lifted.
      }
    }
  }
}

TEST_P(MultilaneLanesParamTest, FlatArcLane) {
  RoadGeometry rg(api::RoadGeometryId{"apple"}, kLinearTolerance, kAngularTolerance, kScaleLength);
  const double theta0 = 0.25 * M_PI;
  const double d_theta = 1.5 * M_PI;
  const double radius = 100.;
  const math::Vector2 center{100., -75.};
  const double offset_radius = radius - r0;

  std::unique_ptr<RoadCurve> road_curve_1 = std::make_unique<ArcRoadCurve>(
      center, radius, theta0, d_theta, zp, zp, kLinearTolerance, kScaleLength, kComputationPolicy);
  Segment* s1 = rg.NewJunction(api::JunctionId{"j1"})
                    ->NewSegment(api::SegmentId{"s1"}, std::move(road_curve_1), -kHalfWidth + r0, kHalfWidth + r0,
                                 {0., kMaxHeight});
  Lane* l2 = s1->NewLane(api::LaneId{"l2"}, r0, {-kHalfLaneWidth, kHalfLaneWidth});

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_EQ(l2->id(), api::LaneId("l2"));
  EXPECT_EQ(l2->segment(), s1);
  EXPECT_EQ(l2->index(), 0);
  EXPECT_EQ(l2->to_left(), nullptr);
  EXPECT_EQ(l2->to_right(), nullptr);
  EXPECT_EQ(l2->r0(), r0);

  EXPECT_NEAR(l2->length(), offset_radius * d_theta, kVeryExact);

  EXPECT_TRUE(
      api::test::IsRBoundsClose(l2->lane_bounds(0.), api::RBounds(-kHalfLaneWidth, kHalfLaneWidth), kVeryExact));
  EXPECT_TRUE(api::test::IsRBoundsClose(l2->segment_bounds(0.), api::RBounds(-kHalfWidth, kHalfWidth), kVeryExact));
  EXPECT_TRUE(api::test::IsHBoundsClose(l2->elevation_bounds(0., 0.), api::HBounds(0., kMaxHeight), kVeryExact));
  // Recall that the arc has center (100, -75).
  const math::Vector3 inertial_center(100., -75., 0.);
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      l2->ToInertialPosition({0., 0., 0.}),
      api::InertialPosition::FromXyz(inertial_center +
                                     math::Vector3(std::cos(theta0), std::sin(theta0), 0.0) * offset_radius),
      kLinearTolerance));

  EXPECT_TRUE(api::test::IsInertialPositionClose(
      l2->ToInertialPosition({1., 0., 0.}),
      api::InertialPosition::FromXyz(inertial_center +
                                     math::Vector3(offset_radius * std::cos(theta0 + 1.0 / offset_radius),
                                                   offset_radius * std::sin(theta0 + 1.0 / offset_radius), 0.0)),
      kLinearTolerance));

  EXPECT_TRUE(api::test::IsInertialPositionClose(
      l2->ToInertialPosition({0., 1., 0.}),
      api::InertialPosition::FromXyz(inertial_center + math::Vector3((offset_radius - 1.) * std::cos(theta0),
                                                                     (offset_radius - 1.) * std::sin(theta0), 0.0)),
      kLinearTolerance));

  EXPECT_TRUE(api::test::IsInertialPositionClose(
      l2->ToInertialPosition({l2->length(), 0., 0.}),
      api::InertialPosition::FromXyz(inertial_center + math::Vector3(offset_radius * std::cos(theta0 + d_theta),
                                                                     offset_radius * std::sin(theta0 + d_theta), 0.0)),
      kLinearTolerance));

  // Case 1: Tests ArcLane::ToSegmentPosition() with a closest point that lies
  // within the lane bounds.
  const api::InertialPosition point_within_lane{center[0] - 50., center[1] + 50., 0.};  // θ = 0.5π.
  const double expected_s = 0.5 * M_PI / d_theta * l2->length();
  const double expected_r = std::min(offset_radius - std::sqrt(2) * 50., kHalfWidth);
  api::LanePositionResult result = l2->ToSegmentPosition(point_within_lane);
  EXPECT_TRUE(
      api::test::IsLanePositionClose(result.lane_position, api::LanePosition(expected_s, expected_r, 0.), kVeryExact));
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position,
      api::InertialPosition::FromXyz(inertial_center +
                                     math::Vector3((offset_radius - kHalfWidth) * std::cos(0.5 * M_PI + theta0),
                                                   (offset_radius - kHalfWidth) * std::sin(0.5 * M_PI + theta0), 0.)),
      kVeryExact));
  EXPECT_NEAR(result.distance, (offset_radius - kHalfWidth) - std::sqrt(std::pow(50., 2.) + std::pow(50., 2.)),
              kVeryExact);

  // Case 2: Tests ArcLane::ToSegmentPosition() with a closest point that lies
  // outside of the lane bounds, verifying that the result saturates.
  const api::InertialPosition point_outside_lane{center[0] + 200., center[1] - 20., 20.};  // θ ~= 1.9π.
  const double expected_r_outside = -kHalfWidth;
  result = l2->ToSegmentPosition(point_outside_lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(
      result.lane_position, api::LanePosition(l2->length(), expected_r_outside, kMaxHeight), kVeryExact));
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position,
      api::InertialPosition::FromXyz(
          inertial_center + math::Vector3((offset_radius + kHalfWidth) * std::cos(theta0 + d_theta),
                                          (offset_radius + kHalfWidth) * std::sin(theta0 + d_theta), kMaxHeight)),
      kVeryExact));
  EXPECT_DOUBLE_EQ(result.distance, (result.nearest_position.xyz() - point_outside_lane.xyz()).norm());

  // Case 3: Tests ArcLane::ToSegmentPosition() at a non-zero but flat elevation.
  const double elevation = 10.;
  std::unique_ptr<RoadCurve> road_curve_2 = std::make_unique<ArcRoadCurve>(
      center, radius, theta0, d_theta, CubicPolynomial(elevation / radius / d_theta, 0.0, 0.0, 0.0), zp,
      kLinearTolerance, kScaleLength, kComputationPolicy);
  Segment* s2 = rg.NewJunction(api::JunctionId{"j2"})
                    ->NewSegment(api::SegmentId{"s2"}, std::move(road_curve_2), -kHalfWidth + r0, kHalfWidth + r0,
                                 {0., kMaxHeight});
  Lane* l2_with_z = s2->NewLane(api::LaneId{"l2_with_z"}, r0, {-kHalfLaneWidth, kHalfLaneWidth});
  result = l2_with_z->ToSegmentPosition(point_outside_lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(
      result.lane_position, api::LanePosition(l2_with_z->length(), expected_r_outside, kMaxHeight), kVeryExact));
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position,
      api::InertialPosition::FromXyz(inertial_center +
                                     math::Vector3((offset_radius + kHalfWidth) * std::cos(theta0 + d_theta),
                                                   (offset_radius + kHalfWidth) * std::sin(theta0 + d_theta),
                                                   kMaxHeight + elevation)),
      kVeryExact));
  EXPECT_DOUBLE_EQ(result.distance, (result.nearest_position.xyz() - point_outside_lane.xyz()).norm());

  // Case 4: Tests ArcLane::ToSegmentPosition() with a lane that overlaps itself.
  // The result should be identical to Case 1.
  const double d_theta_overlap = 3 * M_PI;
  std::unique_ptr<RoadCurve> road_curve_3 = std::make_unique<ArcRoadCurve>(
      center, radius, theta0, d_theta_overlap, zp, zp, kLinearTolerance, kScaleLength, kComputationPolicy);
  Segment* s3 = rg.NewJunction(api::JunctionId{"j3"})
                    ->NewSegment(api::SegmentId{"s3"}, std::move(road_curve_3), -kHalfWidth + r0, kHalfWidth + r0,
                                 {0., kMaxHeight});
  Lane* l2_overlapping = s3->NewLane(api::LaneId{"l2_overlapping"}, r0, {-kHalfLaneWidth, kHalfLaneWidth});
  result = l2_overlapping->ToSegmentPosition(point_within_lane);
  EXPECT_TRUE(
      api::test::IsLanePositionClose(result.lane_position, api::LanePosition(expected_s, expected_r, 0.), kVeryExact));
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      result.nearest_position,
      api::InertialPosition::FromXyz(inertial_center +
                                     math::Vector3((offset_radius - kHalfWidth) * std::cos(0.5 * M_PI + theta0),
                                                   (offset_radius - kHalfWidth) * std::sin(0.5 * M_PI + theta0), 0.)),
      kVeryExact));

  EXPECT_NEAR(result.distance, (offset_radius - kHalfWidth) - std::sqrt(std::pow(50., 2.) + std::pow(50., 2.)),
              kVeryExact);

  // Case 5: Tests ArcLane::ToLanePosition() with a lane that starts in the
  // third quadrant and ends in the second quadrant; i.e. d_theta is negative
  // and crosses the ±π wrap-around value using a point that is within the lane
  // in the third quadrant.
  const double theta0_wrap = 1.2 * M_PI;
  const double d_theta_wrap = -0.4 * M_PI;
  std::unique_ptr<RoadCurve> road_curve_4 = std::make_unique<ArcRoadCurve>(
      center, radius, theta0_wrap, d_theta_wrap, zp, zp, kLinearTolerance, kScaleLength, kComputationPolicy);
  Segment* s4 = rg.NewJunction(api::JunctionId{"j4"})
                    ->NewSegment(api::SegmentId{"s4"}, std::move(road_curve_4), -kHalfWidth + r0, kHalfWidth + r0,
                                 {0., kMaxHeight});
  Lane* l2_wrap = s4->NewLane(api::LaneId{"l2_wrap"}, r0, {-kHalfLaneWidth, kHalfLaneWidth});
  const double offset_radius_wrap = radius + r0 + 2.;
  const api::InertialPosition point_in_third_quadrant{// θ ~= -0.9π.
                                                      center[0] + offset_radius_wrap * std::cos(-0.9 * M_PI),
                                                      center[1] + offset_radius_wrap * std::sin(-0.9 * M_PI), 0.};
  const double expected_s_wrap = std::abs(0.1 * M_PI / d_theta_wrap) * l2_wrap->length();
  const double expected_r_wrap = 2.;
  result = l2_wrap->ToLanePosition(point_in_third_quadrant);
  EXPECT_TRUE(api::test::IsLanePositionClose(result.lane_position,
                                             api::LanePosition(expected_s_wrap, expected_r_wrap, 0.), kVeryExact));
  EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, point_in_third_quadrant, kVeryExact));

  EXPECT_NEAR(result.distance, 0. /* within lane */, kVeryExact);

  // Verifies the output of ArcLane::GetOrientation().
  EXPECT_TRUE(api::test::IsRotationClose(l2->GetOrientation({0., 0., 0.}),
                                         api::Rotation::FromRpy(0., 0., (0.25 + 0.5) * M_PI), kVeryExact));

  EXPECT_TRUE(api::test::IsRotationClose(l2->GetOrientation({0., 1., 0.}),
                                         api::Rotation::FromRpy(0., 0., (0.25 + 0.5) * M_PI), kVeryExact));

  EXPECT_TRUE(api::test::IsRotationClose(l2->GetOrientation({l2->length(), 0., 0.}),
                                         api::Rotation::FromRpy(0., 0, 0.25 * M_PI),
                                         kVeryExact));  // 0.25 + 1.5 + 0.5

  // For r=0, derivative map should be identity.
  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
                                             api::LanePosition(0., 0., 0.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
                                             api::LanePosition(1., 0., 0.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
                                             api::LanePosition(0., 1., 0.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
                                             api::LanePosition(0., 0., 1.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({l2->length(), 0., 0.}, {1., 1., 1.}),
                                             api::LanePosition(1., 1., 1.), kVeryExact));
  // For a left-turning curve, r = +10 will decrease the radius of the path
  // from the original 100 down to 90.
  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., 10., 0.}, {1., 1., 1.}),
                                             api::LanePosition((offset_radius / (offset_radius - 10.0)) * 1., 1., 1.),
                                             kVeryExact));
  // Likewise, r = -10 will increase the radius of the path from the
  // original 100 up to 110.
  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., -10., 0.}, {1., 1., 1.}),
                                             api::LanePosition(offset_radius / (offset_radius + 10.) * 1., 1., 1.),
                                             kVeryExact));
  // ...and only r should matter for an otherwise flat arc.
  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({l2->length(), -10., 100.}, {1., 1., 1.}),
                                             api::LanePosition(offset_radius / (offset_radius + 10.), 1., 1.),
                                             kVeryExact));
}

api::LanePosition IntegrateTrivially(const api::Lane* lane, const api::LanePosition& lp_initial,
                                     const api::IsoLaneVelocity& velocity, const double time_step,
                                     const int step_count) {
  api::LanePosition lp_current = lp_initial;

  for (int i = 0; i < step_count; ++i) {
    const api::LanePosition lp_dot = lane->EvalMotionDerivatives(lp_current, velocity);
    lp_current.set_srh(lp_current.srh() + (lp_dot.srh() * time_step));
  }
  return lp_current;
}

// Checks EvalMotionDerivatives() correctness on a non-trivial road surface.
TEST_P(MultilaneLanesParamTest, HillIntegration) {
  // Sets up a single segment road with three (3) adjacent lanes, whose
  // reference curve is a piece of an arc with cubic elevation and zero
  // superelevation.
  RoadGeometry rg(api::RoadGeometryId{"apple"}, kLinearTolerance, kAngularTolerance, kScaleLength);
  const double theta0 = 0.25 * M_PI;
  const double d_theta = 0.5 * M_PI;
  const double theta1 = theta0 + d_theta;
  const double radius = 100.;
  const double offset_radius = radius - r0;
  const double l_max = radius * d_theta;
  const double z0 = 0.;
  const double z1 = 20.;
  // A cubic polynomial such that:
  //   f(0) = (z0 / l_max),
  ///  f(1) = (z1 / l_max),
  //   and f'(0) = f'(1) = 0.
  const CubicPolynomial kHillPolynomial(z0 / l_max, 0., (3. * (z1 - z0) / l_max), (-2. * (z1 - z0) / l_max));
  std::unique_ptr<RoadCurve> road_curve =
      std::make_unique<ArcRoadCurve>(math::Vector2(-100., -100.), radius, theta0, d_theta, kHillPolynomial, zp,
                                     kLinearTolerance, kScaleLength, kComputationPolicy);
  const double kLaneSpacing = 2. * kHalfLaneWidth;
  const double kLeftWidth = kLaneSpacing + kHalfLaneWidth;
  const double kRightWidth = kLaneSpacing + kHalfLaneWidth;
  Junction* junction = rg.NewJunction(api::JunctionId{"junction"});
  Segment* segment = junction->NewSegment(api::SegmentId{"segment"}, std::move(road_curve), -kLeftWidth + r0,
                                          r0 + kRightWidth, {0., kMaxHeight});
  Lane* right_lane = segment->NewLane(api::LaneId{"right_lane"}, r0 - kLaneSpacing, {-kHalfLaneWidth, kHalfLaneWidth});
  Lane* center_lane = segment->NewLane(api::LaneId{"center_lane"}, r0, {-kHalfLaneWidth, kHalfLaneWidth});
  Lane* left_lane = segment->NewLane(api::LaneId{"left_lane"}, r0 + kLaneSpacing, {-kHalfLaneWidth, kHalfLaneWidth});
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  // Checks center lane endpoints' position in the world frame
  // against their analytically known values.
  const api::LanePosition kInitialLanePosition{0., 0., 0.};
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      center_lane->ToInertialPosition(kInitialLanePosition),
      api::InertialPosition(-100. + (offset_radius * std::cos(theta0)), -100. + (offset_radius * std::sin(theta0)), z0),
      kLinearTolerance));

  const api::LanePosition kFinalLanePosition{center_lane->length(), 0., 0.};
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      center_lane->ToInertialPosition(kFinalLanePosition),
      api::InertialPosition(-100. + (offset_radius * std::cos(theta1)), -100. + (offset_radius * std::sin(theta1)), z1),
      kLinearTolerance));

  // Checks EvalMotionDerivatives() accuracy. To that end, motion derivatives
  // are (1) queried for a given constant velocity σᵥ from the center lane at an
  // r-offset that matches that of an adjacent lane centerline, (2) integrated
  // using Euler's method with a fixed time step dt and (3) compared against the
  // center lane's length. Total integration steps count n is estimated based
  // on the corresponding adjacent lane's length l and velocity σᵥ as
  // n = l / (σᵥ * dt). Path length estimation error is expected to be higher
  // than that achievable by a RoadCurve instance.
  const double kTimeStep{0.001};
  const api::IsoLaneVelocity kVelocity{1., 0., 0.};
  const double kIntegrationTolerance{1e-3};

  // Tests using the lane on the right side.
  const int kStepCountR = right_lane->length() / (kVelocity.sigma_v * kTimeStep);
  const api::LanePosition kInitialLanePositionR{0., -kLaneSpacing, 0.};
  const api::LanePosition kExpectedFinalLanePositionR{center_lane->length(), -kLaneSpacing, 0.};
  EXPECT_TRUE(api::test::IsLanePositionClose(
      IntegrateTrivially(center_lane, kInitialLanePositionR, kVelocity, kTimeStep, kStepCountR),
      kExpectedFinalLanePositionR, kIntegrationTolerance));

  // Tests using the lane on the left side.
  const int kStepCountL = left_lane->length() / (kVelocity.sigma_v * kTimeStep);
  const api::LanePosition kInitialLanePositionL{0., kLaneSpacing, 0.};
  const api::LanePosition kExpectedFinalLanePositionL{center_lane->length(), kLaneSpacing, 0.};
  EXPECT_TRUE(api::test::IsLanePositionClose(
      IntegrateTrivially(center_lane, kInitialLanePositionL, kVelocity, kTimeStep, kStepCountL),
      kExpectedFinalLanePositionL, kIntegrationTolerance));
}

INSTANTIATE_TEST_CASE_P(Offset, MultilaneLanesParamTest, testing::Values(0., 5., -5.));

GTEST_TEST(MultilaneLanesTest, ArcLaneWithConstantSuperelevation) {
  CubicPolynomial zp{0., 0., 0., 0.};
  const double kScaleLength{1.};
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};
  const double kTheta = 0.10 * M_PI;  // superelevation
  const double kR0 = 0.;
  const double kHalfWidth = 10.;
  const double kHalfLaneWidth = 5.;
  const double kMaxHeight = 5.;

  RoadGeometry rg(api::RoadGeometryId{"apple"}, kLinearTolerance, kAngularTolerance, kScaleLength);
  std::unique_ptr<RoadCurve> road_curve_1 = std::make_unique<ArcRoadCurve>(
      math::Vector2(100., -75.), 100.0, 0.25 * M_PI, 1.5 * M_PI, zp,
      CubicPolynomial((kTheta) / (100. * 1.5 * M_PI), 0., 0., 0.), kLinearTolerance, kScaleLength, kComputationPolicy);
  Segment* s1 = rg.NewJunction(api::JunctionId{"j1"})
                    ->NewSegment(api::SegmentId{"s1"}, std::move(road_curve_1), -kHalfWidth + kR0, kHalfWidth + kR0,
                                 {0., kMaxHeight});
  Lane* l2 = s1->NewLane(api::LaneId{"l2"}, kR0, {-kHalfLaneWidth, kHalfLaneWidth});

  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  EXPECT_NEAR(l2->length(), 100. * 1.5 * M_PI, kVeryExact);

  EXPECT_TRUE(api::test::IsInertialPositionClose(
      l2->ToInertialPosition({0., 0., 0.}),
      api::InertialPosition(100. + (100. * std::cos(0.25 * M_PI)), -75. + (100. * std::sin(0.25 * M_PI)), 0.),
      kLinearTolerance));

  // NB: (1.25 * M_PI) is the direction of the r-axis at s = 0.
  EXPECT_TRUE(api::test::IsInertialPositionClose(
      l2->ToInertialPosition({0., 10., 0.}),
      api::InertialPosition(100. + (100. * std::cos(0.25 * M_PI)) + (10. * std::cos(kTheta) * std::cos(1.25 * M_PI)),
                            -75. + (100. * std::sin(0.25 * M_PI)) + (10. * std::cos(kTheta) * std::sin(1.25 * M_PI)),
                            10. * std::sin(kTheta)),
      kLinearTolerance));

  // TODO(maddog@tri.global) Test ToLanePosition().

  EXPECT_TRUE(api::test::IsRotationClose(l2->GetOrientation({0., 0., 0.}),
                                         api::Rotation::FromRpy(kTheta, 0., (0.25 + 0.5) * M_PI), kVeryExact));

  EXPECT_TRUE(api::test::IsRotationClose(l2->GetOrientation({0., 1., 0.}),
                                         api::Rotation::FromRpy(kTheta, 0., (0.25 + 0.5) * M_PI), kVeryExact));

  EXPECT_TRUE(api::test::IsRotationClose(l2->GetOrientation({l2->length(), 0., 0.}),
                                         api::Rotation::FromRpy(kTheta, 0., 0.25 * M_PI),
                                         kVeryExact));  // 0.25 + 1.5 + 0.5

  api::LanePosition pdot;
  // For r=0, derivative map should be identity.
  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
                                             api::LanePosition(0., 0., 0.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
                                             api::LanePosition(1., 0., 0.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
                                             api::LanePosition(0., 1., 0.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
                                             api::LanePosition(0., 0., 1.), kVeryExact));

  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({l2->length(), 0., 0.}, {1., 1., 1.}),
                                             api::LanePosition(1., 1., 1.), kVeryExact));

  // For a left-turning curve, r = +10 will decrease the radius of the path
  // from the original 100 down to almost 90.  (r is scaled by the cosine of
  // the superelevation since it is no longer horizontal).
  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., 10., 0.}, {1., 1., 1.}),
                                             api::LanePosition((100. / (100. - (10. * std::cos(kTheta)))) * 1., 1., 1.),
                                             kVeryExact));
  // Likewise, r = -10 will increase the radius of the path from the
  // original 100 up to almost 110 (since r is no longer horizontal).
  EXPECT_TRUE(api::test::IsLanePositionClose(l2->EvalMotionDerivatives({0., -10., 0.}, {1., 1., 1.}),
                                             api::LanePosition((100. / (100 + (10. * std::cos(kTheta)))) * 1., 1., 1.),
                                             kVeryExact));

  // h matters, too (because hovering above a tilted road changes one's
  // distance to the center of the arc).
  EXPECT_TRUE(api::test::IsLanePositionClose(
      l2->EvalMotionDerivatives({l2->length(), -10., 8.}, {1., 1., 1.}),
      api::LanePosition((100. / (100 + (10. * std::cos(kTheta)) + (8. * std::sin(kTheta)))) * 1., 1., 1.), kVeryExact));
}

class MultilaneMultipleLanesTest : public ::testing::Test {
 protected:
  const CubicPolynomial zp{0., 0., 0., 0.};
  const double kScaleLength{1.};
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};
  const double kR0{10.};
  const double kRSpacing{15.};
  const double kRMin{2.};
  const double kRMax{42.};
  const double kHalfLaneWidth{0.5 * kRSpacing};
  const double kMaxHeight{5.};
  const api::HBounds height_bounds{0.0, kMaxHeight};
};

TEST_F(MultilaneMultipleLanesTest, MultipleLineLanes) {
  RoadGeometry rg(api::RoadGeometryId{"apple"}, kLinearTolerance, kAngularTolerance, kScaleLength);
  std::unique_ptr<RoadCurve> road_curve = std::make_unique<LineRoadCurve>(
      math::Vector2(100., -75.), math::Vector2(100., 50.), zp, zp, kLinearTolerance, kScaleLength, kComputationPolicy);
  Segment* s1 = rg.NewJunction(api::JunctionId{"j1"})
                    ->NewSegment(api::SegmentId{"s1"}, std::move(road_curve), kRMin, kRMax, height_bounds);
  const Lane* l0 = s1->NewLane(api::LaneId{"l0"}, kR0, {-8., kHalfLaneWidth});
  const Lane* l1 = s1->NewLane(api::LaneId{"l1"}, kR0 + kRSpacing, {-kHalfLaneWidth, kHalfLaneWidth});
  const Lane* l2 = s1->NewLane(api::LaneId{"l2"}, kR0 + 2. * kRSpacing, {-kHalfLaneWidth, 2.});
  const std::vector<const Lane*> lanes{l0, l1, l2};

  // Checks r0.
  EXPECT_EQ(l0->r0(), kR0);
  EXPECT_EQ(l1->r0(), kR0 + kRSpacing);
  EXPECT_EQ(l2->r0(), kR0 + 2. * kRSpacing);

  // Checks right to left ordering.
  EXPECT_EQ(l0->to_right(), nullptr);
  EXPECT_EQ(l0->to_left(), l1);
  EXPECT_EQ(l1->to_right(), l0);
  EXPECT_EQ(l1->to_left(), l2);
  EXPECT_EQ(l2->to_right(), l1);
  EXPECT_EQ(l2->to_left(), nullptr);

  // Checks length of the three Lanes.
  const double lane_length = math::Vector2(100., 50.).norm();
  for (const Lane* lane : lanes) {
    EXPECT_EQ(lane->length(), lane_length);
  }

  // Checks api::InertialPositions and api::Rotations at different points over the
  // centerline of each Lane.
  const math::Vector3 s_vector = math::Vector3(100., 50., 0.).normalized();
  const math::Vector3 r_vector = math::Vector3(-50, 100., 0.).normalized();
  const std::vector<double> p_vector{0., 0.1, 0.2, 0.5, 0.7, 1.0};
  const std::vector<double> r_offset_vector{-2., 0., 2.};
  double lane_spacing{};
  const double theta = std::atan2(50., 100.);
  for (const Lane* lane : lanes) {
    for (const double p : p_vector) {
      for (const double r : r_offset_vector) {
        EXPECT_TRUE(api::test::IsInertialPositionClose(
            lane->ToInertialPosition({p * lane_length, r, 0.}),
            api::InertialPosition::FromXyz(math::Vector3(100., -75., 0.) + (p * lane_length) * s_vector +
                                           (kR0 + lane_spacing + r) * r_vector),
            kVeryExact));
        EXPECT_TRUE(api::test::IsRotationClose(lane->GetOrientation({p * lane_length, r, 0.}),
                                               api::Rotation::FromRpy(0., 0., theta), kVeryExact));
      }
    }
    lane_spacing += kRSpacing;
  }

  // Checks api::LanePosition conversion to api::InertialPosition in the
  // surroundings of each Lane. No saturation is tested.
  lane_spacing = 0.;
  for (const Lane* lane : lanes) {
    for (const double p : p_vector) {
      for (const double r : r_offset_vector) {
        const api::InertialPosition inertial_point = api::InertialPosition::FromXyz(
            math::Vector3(100., -75., 0.) + (p * lane_length) * s_vector + (kR0 + lane_spacing + r) * r_vector);

        const api::LanePositionResult result = lane->ToSegmentPosition(inertial_point);
        EXPECT_TRUE(api::test::IsLanePositionClose(result.lane_position, api::LanePosition(p * lane_length, r, 0.),
                                                   kVeryExact));
        EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, inertial_point, kVeryExact));
        EXPECT_NEAR(result.distance, 0., kVeryExact);
      }
    }
    lane_spacing += kRSpacing;
  }

  // Checks api::LanePosition conversion to api::InertialPosition in the Segment's
  // RoadCurve, which is outside the segment bounds, to verify that it
  // saturates.
  for (const Lane* lane : lanes) {
    for (const double p : p_vector) {
      const api::InertialPosition inertial_point =
          api::InertialPosition::FromXyz(math::Vector3(100., -75., 0.) + (p * lane_length) * s_vector);
      const double expected_r = lane->segment_bounds(0.).min();
      const api::LanePositionResult result = lane->ToSegmentPosition(inertial_point);
      EXPECT_TRUE(api::test::IsLanePositionClose(result.lane_position,
                                                 api::LanePosition(p * lane_length, expected_r, 0.), kVeryExact));
      EXPECT_TRUE(api::test::IsInertialPositionClose(
          result.nearest_position,
          api::InertialPosition::FromXyz(math::Vector3(100., -75., 0.) + (p * lane_length) * s_vector +
                                         kRMin * r_vector),
          kVeryExact));
      EXPECT_NEAR(result.distance, kRMin, kVeryExact);
    }
  }

  // Derivative map should be identity (for a flat, straight road).
  for (const Lane* lane : lanes) {
    EXPECT_TRUE(api::test::IsLanePositionClose(lane->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
                                               api::LanePosition(0., 0., 0.), kVeryExact));

    EXPECT_TRUE(api::test::IsLanePositionClose(lane->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
                                               api::LanePosition(1., 0., 0.), kVeryExact));

    EXPECT_TRUE(api::test::IsLanePositionClose(lane->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
                                               api::LanePosition(0., 1., 0.), kVeryExact));

    EXPECT_TRUE(api::test::IsLanePositionClose(lane->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
                                               api::LanePosition(0., 0., 1.), kVeryExact));

    EXPECT_TRUE(api::test::IsLanePositionClose(l1->EvalMotionDerivatives({10., 5., 3.}, {1., 2., 3.}),
                                               api::LanePosition(1., 2., 3.), kVeryExact));
  }
}

TEST_F(MultilaneMultipleLanesTest, MultipleArcLanes) {
  const double kTheta0{0.25 * M_PI};
  const double kDTheta{1.5 * M_PI};
  const double kRadius{100.};
  const math::Vector2 kCenter{100., -75.};
  const math::Vector3 kGeoCenter{kCenter[0], kCenter[1], 0.};

  RoadGeometry rg(api::RoadGeometryId{"apple"}, kLinearTolerance, kAngularTolerance, kScaleLength);
  std::unique_ptr<RoadCurve> road_curve = std::make_unique<ArcRoadCurve>(
      kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kScaleLength, kComputationPolicy);
  Segment* s1 = rg.NewJunction(api::JunctionId{"j1"})
                    ->NewSegment(api::SegmentId{"s1"}, std::move(road_curve), kRMin, kRMax, height_bounds);
  const Lane* l0 = s1->NewLane(api::LaneId{"l0"}, kR0, {-8., kHalfLaneWidth});
  const Lane* l1 = s1->NewLane(api::LaneId{"l1"}, kR0 + kRSpacing, {-kHalfLaneWidth, kHalfLaneWidth});
  const Lane* l2 = s1->NewLane(api::LaneId{"l2"}, kR0 + 2. * kRSpacing, {-kHalfLaneWidth, 2.});
  const std::vector<const Lane*> lanes{l0, l1, l2};

  // Checks r0.
  EXPECT_EQ(l0->r0(), kR0);
  EXPECT_EQ(l1->r0(), kR0 + kRSpacing);
  EXPECT_EQ(l2->r0(), kR0 + 2. * kRSpacing);

  // Checks right to left ordering.
  EXPECT_EQ(l0->to_right(), nullptr);
  EXPECT_EQ(l0->to_left(), l1);
  EXPECT_EQ(l1->to_right(), l0);
  EXPECT_EQ(l1->to_left(), l2);
  EXPECT_EQ(l2->to_right(), l1);
  EXPECT_EQ(l2->to_left(), nullptr);

  // Checks length of the three Lanes.
  const std::vector<double> lane_radius{(kRadius - kR0), (kRadius - kR0 - kRSpacing), (kRadius - kR0 - 2. * kRSpacing)};
  for (size_t i = 0; i < lanes.size(); i++) {
    EXPECT_EQ(lanes[i]->length(), lane_radius[i] * kDTheta);
  }

  // Checks api::InertialPositions and api::Rotations at different points over the
  // centerline of each Lane.
  auto wrap_angle = [](double angle) {
    double angle_new = std::fmod(angle + M_PI, 2. * M_PI);
    if (angle_new < 0.) angle_new += 2. * M_PI;
    return angle_new - M_PI;
  };
  const std::vector<double> p_vector{0., 0.1, 0.2, 0.5, 0.7, 1.0};
  const std::vector<double> r_offset_vector{-2., 0., 2.};
  for (size_t i = 0; i < lanes.size(); i++) {
    const Lane* lane = lanes[i];
    const double radius = lane_radius[i];
    for (const double p : p_vector) {
      for (const double r : r_offset_vector) {
        const double effective_radius = radius - r;
        const double effective_angle = p * kDTheta + kTheta0;
        EXPECT_TRUE(api::test::IsInertialPositionClose(
            lane->ToInertialPosition({p * radius * kDTheta, r, 0.}),
            api::InertialPosition::FromXyz(kGeoCenter + effective_radius * math::Vector3(std::cos(effective_angle),
                                                                                         std::sin(effective_angle),
                                                                                         0.)),
            kVeryExact));
        EXPECT_TRUE(api::test::IsRotationClose(
            lane->GetOrientation({p * radius * kDTheta, r, 0.}),
            api::Rotation::FromRpy(0., 0., wrap_angle(effective_angle + (0.5 * M_PI))), kVeryExact));
      }
    }
  }

  // Checks api::LanePosition conversion to api::InertialPosition in the
  // surroundings of each Lane. No saturation is tested.
  for (size_t i = 0; i < lanes.size(); i++) {
    const Lane* lane = lanes[i];
    const double radius = lane_radius[i];
    for (const double p : p_vector) {
      for (const double r : r_offset_vector) {
        const double effective_radius = radius - r;
        const double effective_angle = p * kDTheta + kTheta0;
        const api::InertialPosition inertial_point = api::InertialPosition::FromXyz(
            kGeoCenter + effective_radius * math::Vector3(std::cos(effective_angle), std::sin(effective_angle), 0.));
        const api::LanePositionResult result = lane->ToSegmentPosition(inertial_point);
        EXPECT_TRUE(api::test::IsLanePositionClose(result.lane_position, api::LanePosition(p * radius * kDTheta, r, 0.),
                                                   kVeryExact));
        EXPECT_TRUE(api::test::IsInertialPositionClose(result.nearest_position, inertial_point, kVeryExact));
        EXPECT_NEAR(result.distance, 0., kVeryExact);
      }
    }
  }

  // Checks api::LanePosition conversion to api::InertialPosition in the Segment's
  // RoadCurve, which is outside the segment bounds, to verify that it
  // saturates.
  for (size_t i = 0; i < lanes.size(); i++) {
    const Lane* lane = lanes[i];
    const double radius = lane_radius[i];
    for (const double p : p_vector) {
      const double effective_angle = p * kDTheta + kTheta0;
      const api::InertialPosition inertial_point = api::InertialPosition::FromXyz(
          kGeoCenter + kRadius * math::Vector3(std::cos(effective_angle), std::sin(effective_angle), 0.));
      const double expected_r = lane->segment_bounds(0.).min();
      const api::LanePositionResult result = lane->ToSegmentPosition(inertial_point);
      EXPECT_TRUE(api::test::IsLanePositionClose(result.lane_position,
                                                 api::LanePosition(p * radius * kDTheta, expected_r, 0.), kVeryExact));
      EXPECT_TRUE(api::test::IsInertialPositionClose(
          result.nearest_position,
          api::InertialPosition::FromXyz(
              kGeoCenter + (kRadius - kRMin) * math::Vector3(std::cos(effective_angle), std::sin(effective_angle), 0.)),
          kVeryExact));
      EXPECT_NEAR(result.distance, kRMin, kVeryExact);
    }
  }

  // Checks motion derivatives for the lanes.
  for (size_t i = 0; i < lanes.size(); i++) {
    const Lane* lane = lanes[i];
    const double radius = lane_radius[i];

    // For r=0, derivative map should be identity.
    EXPECT_TRUE(api::test::IsLanePositionClose(lane->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 0.}),
                                               api::LanePosition(0., 0., 0.), kVeryExact));

    EXPECT_TRUE(api::test::IsLanePositionClose(lane->EvalMotionDerivatives({0., 0., 0.}, {1., 0., 0.}),
                                               api::LanePosition(1., 0., 0.), kVeryExact));

    EXPECT_TRUE(api::test::IsLanePositionClose(lane->EvalMotionDerivatives({0., 0., 0.}, {0., 1., 0.}),
                                               api::LanePosition(0., 1., 0.), kVeryExact));

    EXPECT_TRUE(api::test::IsLanePositionClose(lane->EvalMotionDerivatives({0., 0., 0.}, {0., 0., 1.}),
                                               api::LanePosition(0., 0., 1.), kVeryExact));

    EXPECT_TRUE(api::test::IsLanePositionClose(lane->EvalMotionDerivatives({lane->length(), 0., 0.}, {1., 1., 1.}),
                                               api::LanePosition(1., 1., 1.), kVeryExact));

    // Checks motion derivatives at different offsets from Lane's centerline.
    for (const double r : r_offset_vector) {
      EXPECT_TRUE(api::test::IsLanePositionClose(lane->EvalMotionDerivatives({0., r, 0.}, {1., 1., 1.}),
                                                 api::LanePosition((radius / (radius - r)) * 1., 1., 1.), kVeryExact));
      EXPECT_TRUE(api::test::IsLanePositionClose(lane->EvalMotionDerivatives({lane->length(), r, 100.}, {1., 1., 1.}),
                                                 api::LanePosition(radius / (radius - r), 1., 1.), kVeryExact));
    }
  }
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
