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
#include "maliput_multilane/arc_road_curve.h"
/* clang-format on */

#include <gtest/gtest.h>
#include <maliput/api/lane_data.h>
#include <maliput/common/assertion_error.h>
#include <maliput/test_utilities/maliput_math_compare.h>

namespace maliput {
namespace multilane {
namespace {

using maliput::math::test::CompareVectors;

class MultilaneArcRoadCurveTest : public ::testing::Test {
 protected:
  const math::Vector2 kCenter{10.0, 10.0};
  const double kRadius{10.0};
  const double kTheta0{M_PI / 4.0};
  const double kTheta1{3.0 * M_PI / 4.0};
  const double kDTheta{kTheta1 - kTheta0};
  const CubicPolynomial zp;
  const double kZeroTolerance{0.};
  const double kLinearTolerance{0.01};
  const double kZeroScaleLength{0.};
  const double kScaleLength{1.};
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};
  const double kRMin{-0.5 * kRadius};
  const double kRMax{0.5 * kRadius};
  const api::HBounds height_bounds{0.0, 10.0};
  const double kVeryExact{1e-12};
  const double kR0Offset{0.0};
  const double kROffset{5.0};
};

// Checks ArcRoadCurve constructor constraints.
TEST_F(MultilaneArcRoadCurveTest, ConstructorTest) {
  EXPECT_THROW(
      ArcRoadCurve(kCenter, -kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kScaleLength, kComputationPolicy),
      maliput::common::assertion_error);

  EXPECT_THROW(
      ArcRoadCurve(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kZeroTolerance, kScaleLength, kComputationPolicy),
      maliput::common::assertion_error);

  EXPECT_THROW(
      ArcRoadCurve(kCenter, kRadius, kTheta0, kDTheta, zp, zp, -kLinearTolerance, kScaleLength, kComputationPolicy),
      maliput::common::assertion_error);

  EXPECT_THROW(
      ArcRoadCurve(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, -kScaleLength, kComputationPolicy),
      maliput::common::assertion_error);

  EXPECT_THROW(
      ArcRoadCurve(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kZeroScaleLength, kComputationPolicy),
      maliput::common::assertion_error);

  EXPECT_NO_THROW(
      ArcRoadCurve(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kScaleLength, kComputationPolicy));
}

// Checks arc reference curve interpolations, derivatives, and lengths.
TEST_F(MultilaneArcRoadCurveTest, ArcGeometryTest) {
  const ArcRoadCurve dut(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kScaleLength,
                         kComputationPolicy);
  // Checks curve length computations along the centerline.
  const double kExpectedLength = kDTheta * kRadius;
  // The total path length of the reference curve l_max and the total path
  // length of the curve along the centerline s_max for r = h = 0 should match
  // provided that the curve shows no elevation.
  EXPECT_NEAR(dut.l_max(), kExpectedLength, kVeryExact);
  std::function<double(double)> s_from_p_at_r0 = dut.OptimizeCalcSFromP(kR0Offset);
  const double centerline_length = s_from_p_at_r0(1.);
  EXPECT_NEAR(centerline_length, kExpectedLength, kVeryExact);
  // Checks that both `s` and `p` bounds are enforced on
  // mapping evaluation along the centerline
  std::function<double(double)> p_from_s_at_r0 = dut.OptimizeCalcPFromS(kR0Offset);
  EXPECT_THROW(s_from_p_at_r0(2.), maliput::common::assertion_error);
  EXPECT_THROW(p_from_s_at_r0(2. * centerline_length), maliput::common::assertion_error);
  // Checks that both `s` and `p` bounds are enforced on
  // mapping evaluation at an offset
  std::function<double(double)> s_from_p_at_r = dut.OptimizeCalcSFromP(kROffset);
  std::function<double(double)> p_from_s_at_r = dut.OptimizeCalcPFromS(kROffset);
  const double offset_line_length = s_from_p_at_r(1.);
  EXPECT_THROW(s_from_p_at_r(2.), maliput::common::assertion_error);
  EXPECT_THROW(p_from_s_at_r(2. * offset_line_length), maliput::common::assertion_error);

  // Checks the evaluation of xy at different values over the reference curve.
  EXPECT_TRUE(CompareVectors(
      dut.xy_of_p(0.0), kCenter + math::Vector2(kRadius * std::cos(kTheta0), kRadius * std::sin(kTheta0)), kVeryExact));
  EXPECT_TRUE(CompareVectors(
      dut.xy_of_p(0.5),
      kCenter + math::Vector2(kRadius * std::cos(kTheta0 + kDTheta * 0.5), kRadius * std::sin(kTheta0 + kDTheta * 0.5)),
      kVeryExact));
  EXPECT_TRUE(CompareVectors(
      dut.xy_of_p(1.0), kCenter + math::Vector2(kRadius * std::cos(kTheta1), kRadius * std::sin(kTheta1)), kVeryExact));
  // Checks the derivative of xy at different values over the reference curve.
  EXPECT_TRUE(CompareVectors(
      dut.xy_dot_of_p(0.0),
      math::Vector2(-kRadius * std::sin(kTheta0) * kDTheta, kRadius * std::cos(kTheta0) * kDTheta), kVeryExact));
  EXPECT_TRUE(CompareVectors(dut.xy_dot_of_p(0.5),
                             math::Vector2(-kRadius * std::sin(kTheta0 + 0.5 * kDTheta) * kDTheta,
                                           kRadius * std::cos(kTheta0 + 0.5 * kDTheta) * kDTheta),
                             kVeryExact));
  EXPECT_TRUE(CompareVectors(
      dut.xy_dot_of_p(1.0),
      math::Vector2(-kRadius * std::sin(kTheta1) * kDTheta, kRadius * std::cos(kTheta1) * kDTheta), kVeryExact));
  // Checks the heading at different values.
  EXPECT_NEAR(dut.heading_of_p(0.0), kTheta0 + M_PI / 2.0, kVeryExact);
  EXPECT_NEAR(dut.heading_of_p(0.5), kTheta0 + kDTheta / 2.0 + M_PI / 2.0, kVeryExact);
  EXPECT_NEAR(dut.heading_of_p(1.0), kTheta1 + M_PI / 2.0, kVeryExact);
  // Checks the heading derivative of p at different values.
  EXPECT_NEAR(dut.heading_dot_of_p(0.0), kDTheta, kVeryExact);
  EXPECT_NEAR(dut.heading_dot_of_p(0.5), kDTheta, kVeryExact);
  EXPECT_NEAR(dut.heading_dot_of_p(1.0), kDTheta, kVeryExact);
}

// Checks the validity of the surface for different lateral bounds and
// geometries. Test cases are disposed in the following way for each road curve
// and lateral bounds:
// - Inside the lateral bounds.
// - Inside but with smaller lateral bounds.
// - To the right of the bounds.
// - To the left of the bounds.
// - Larger than the radius bounds.
// - Bounds equal to the radius.
GTEST_TEST(MultilaneArcRoadCurve, IsValidTest) {
  const math::Vector2 kZeroCenter(0.0, 0.0);
  const double kRadius = 10.0;
  const double kInitTheta = 0.0;
  const double kEndTheta1 = M_PI / 2.0;
  const double kDTheta1 = kEndTheta1 - kInitTheta;
  const double kEndTheta2 = -M_PI / 2.0;
  const double kDTheta2 = kEndTheta2 - kInitTheta;
  const CubicPolynomial constant_superelevation(M_PI / 4.0, 0.0, 0.0, 0.0);
  const api::HBounds height_bounds(0.0, 10.0);
  const CubicPolynomial zp;
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.};
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};

  // Checks over a flat arc surface.
  const ArcRoadCurve flat_arc_geometry(kZeroCenter, kRadius, kInitTheta, kDTheta1, zp, zp, kLinearTolerance,
                                       kScaleLength, kComputationPolicy);
  EXPECT_TRUE(flat_arc_geometry.IsValid(-0.5 * kRadius, 0.5 * kRadius, height_bounds));
  EXPECT_TRUE(flat_arc_geometry.IsValid(-0.25 * kRadius, 0.25 * kRadius, height_bounds));
  EXPECT_TRUE(flat_arc_geometry.IsValid(0.25 * kRadius, 0.75 * kRadius, height_bounds));
  EXPECT_TRUE(flat_arc_geometry.IsValid(-0.75 * kRadius, -0.25 * kRadius, height_bounds));
  EXPECT_FALSE(flat_arc_geometry.IsValid(-1.5 * kRadius, 1.5 * kRadius, height_bounds));
  EXPECT_FALSE(flat_arc_geometry.IsValid(-kRadius, kRadius, height_bounds));

  // Checks over a right handed cone.
  const ArcRoadCurve right_handed_cone_geometry(kZeroCenter, kRadius, kInitTheta, kDTheta1, zp, constant_superelevation,
                                                kLinearTolerance, kScaleLength, kComputationPolicy);
  EXPECT_TRUE(right_handed_cone_geometry.IsValid(-0.5 * kRadius, 0.5 * kRadius, height_bounds));
  EXPECT_TRUE(right_handed_cone_geometry.IsValid(-0.25 * kRadius, 0.25 * kRadius, height_bounds));
  EXPECT_TRUE(right_handed_cone_geometry.IsValid(0.25 * kRadius, 0.75 * kRadius, height_bounds));
  EXPECT_TRUE(right_handed_cone_geometry.IsValid(-0.75 * kRadius, -0.25 * kRadius, height_bounds));
  EXPECT_FALSE(right_handed_cone_geometry.IsValid(-1.5 * kRadius, 1.5 * kRadius, height_bounds));
  EXPECT_TRUE(right_handed_cone_geometry.IsValid(-kRadius, kRadius, height_bounds));

  // Checks over a left handed cone.
  const ArcRoadCurve left_handed_cone_geometry(kZeroCenter, kRadius, kInitTheta, kDTheta2, zp, constant_superelevation,
                                               kLinearTolerance, kScaleLength, kComputationPolicy);
  EXPECT_TRUE(left_handed_cone_geometry.IsValid(-0.5 * kRadius, 0.5 * kRadius, height_bounds));
  EXPECT_TRUE(left_handed_cone_geometry.IsValid(-0.25 * kRadius, 0.25 * kRadius, height_bounds));
  EXPECT_TRUE(left_handed_cone_geometry.IsValid(0.25 * kRadius, 0.75 * kRadius, height_bounds));
  EXPECT_TRUE(left_handed_cone_geometry.IsValid(-0.75 * kRadius, -0.25 * kRadius, height_bounds));
  EXPECT_FALSE(left_handed_cone_geometry.IsValid(-1.5 * kRadius, 1.5 * kRadius, height_bounds));
  EXPECT_TRUE(left_handed_cone_geometry.IsValid(-kRadius, kRadius, height_bounds));
}

// Checks the ToCurve frame coordinate conversion for different points in world
// coordinates.
TEST_F(MultilaneArcRoadCurveTest, ToCurveFrameTest) {
  const ArcRoadCurve dut(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kScaleLength,
                         kComputationPolicy);
  // Checks points over the composed curve.
  EXPECT_TRUE(CompareVectors(dut.ToCurveFrame(math::Vector3(kCenter[0] + kRadius * std::cos(kTheta0),
                                                            kCenter[1] + kRadius * std::sin(kTheta0), 0.0),
                                              kRMin, kRMax, height_bounds),
                             math::Vector3(0.0, 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(
      CompareVectors(dut.ToCurveFrame(math::Vector3(kCenter[0] + kRadius * std::cos(kTheta0 + kDTheta / 2.0),
                                                    kCenter[1] + kRadius * std::sin(kTheta0 + kDTheta / 2.0), 0.0),
                                      kRMin, kRMax, height_bounds),
                     math::Vector3(0.5, 0.0, 0.0), kVeryExact));
  EXPECT_TRUE(CompareVectors(dut.ToCurveFrame(math::Vector3(kCenter[0] + kRadius * std::cos(kTheta1),
                                                            kCenter[1] + kRadius * std::sin(kTheta1), 0.0),
                                              kRMin, kRMax, height_bounds),
                             math::Vector3(1., 0.0, 0.0), kVeryExact));
  // Checks with lateral and vertical deviations.
  EXPECT_TRUE(
      CompareVectors(dut.ToCurveFrame(math::Vector3(kCenter[0] + (kRadius + 1.0) * std::cos(kTheta0 + M_PI / 8.0),
                                                    kCenter[1] + (kRadius + 1.0) * std::sin(kTheta0 + M_PI / 8.0), 6.0),
                                      kRMin, kRMax, height_bounds),
                     math::Vector3(0.25, -1.0, 6.0), kVeryExact));
  EXPECT_TRUE(CompareVectors(
      dut.ToCurveFrame(
          math::Vector3(kCenter[0] + (kRadius - 2.0) * std::cos(kTheta0 + kDTheta / 2.0 + M_PI / 8.0),
                        kCenter[1] + (kRadius - 2.0) * std::sin(kTheta0 + kDTheta / 2.0 + M_PI / 8.0), 3.0),
          kRMin, kRMax, height_bounds),
      math::Vector3(0.75, 2.0, 3.0), kVeryExact));
}

// Checks that l_max(), p_from_s() and s_from_p() with constant
// superelevation polynomial and up to linear elevation polynomial behave
// properly.
TEST_F(MultilaneArcRoadCurveTest, OffsetTest) {
  const std::vector<double> r_vector{-0.5 * kRadius, 0.0, 0.5 * kRadius};
  const std::vector<double> p_vector{0., 0.1, 0.2, 0.5, 0.7, 1.0};

  // Checks for flat ArcRoadCurve.
  const ArcRoadCurve flat_dut(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kScaleLength,
                              kComputationPolicy);
  EXPECT_DOUBLE_EQ(flat_dut.l_max(), kRadius * kDTheta);
  // Checks that functions throw when lateral offset is exceeded.
  EXPECT_THROW(flat_dut.OptimizeCalcPFromS(kRadius), maliput::common::assertion_error);
  EXPECT_THROW(flat_dut.OptimizeCalcPFromS(2.0 * kRadius), maliput::common::assertion_error);
  EXPECT_THROW(flat_dut.OptimizeCalcPFromS(kRadius), maliput::common::assertion_error);
  EXPECT_THROW(flat_dut.OptimizeCalcSFromP(2.0 * kRadius), maliput::common::assertion_error);
  // Evaluates inverse function for different path length and offset values.
  for (double r : r_vector) {
    std::function<double(double)> p_from_s_at_r = flat_dut.OptimizeCalcPFromS(r);
    for (double p : p_vector) {
      EXPECT_DOUBLE_EQ(p_from_s_at_r(p * (kRadius - r) * kDTheta), p);
    }
  }
  // Evaluates the path length integral for different offset values.
  for (double r : r_vector) {
    std::function<double(double)> s_from_p_at_r = flat_dut.OptimizeCalcSFromP(r);
    for (double p : p_vector) {
      EXPECT_DOUBLE_EQ(s_from_p_at_r(p), p * (kRadius - r) * kDTheta);
    }
  }

  // Checks for linear elevation applied to the ArcRoadCurve.
  const double slope = 10. / (kRadius * kDTheta);
  const CubicPolynomial linear_elevation(10., slope, 0., 0.);
  const ArcRoadCurve elevated_dut(kCenter, kRadius, kTheta0, kDTheta, linear_elevation, zp, kLinearTolerance,
                                  kScaleLength, kComputationPolicy);
  EXPECT_DOUBLE_EQ(elevated_dut.l_max(), kRadius * kDTheta);
  // Evaluates inverse function and path length integral for different values of
  // p and r lateral offsets.
  for (double r : r_vector) {
    std::function<double(double)> s_from_p_at_r = elevated_dut.OptimizeCalcSFromP(r);
    std::function<double(double)> p_from_s_at_r = elevated_dut.OptimizeCalcPFromS(r);
    for (double p : p_vector) {
      const double s = p * kRadius * kDTheta * std::sqrt(std::pow((kRadius - r) / kRadius, 2.) + slope * slope);
      EXPECT_DOUBLE_EQ(p_from_s_at_r(s), p);
      EXPECT_DOUBLE_EQ(s_from_p_at_r(p), s);
    }
  }
}

// Checks World function evaluation at different values of [p, r, h].
TEST_F(MultilaneArcRoadCurveTest, WorldFunction) {
  const std::vector<double> r_vector{-5., 0., 5.};
  const std::vector<double> p_vector{0., 0.1, 0.2, 0.5, 0.7, 1.0};
  const std::vector<double> h_vector{-5., 0., 5.};

  // Checks for a flat curve.
  const ArcRoadCurve flat_dut(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kScaleLength,
                              kComputationPolicy);
  const math::Vector3 kGeoCenter(kCenter.x(), kCenter.y(), 0.);
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        const Rot3 flat_rotation(0., 0., flat_dut.heading_of_p(p));
        const double angle = kTheta0 + kDTheta * p;
        const math::Vector3 geo_position = kGeoCenter + kRadius * math::Vector3(std::cos(angle), std::sin(angle), 0.) +
                                           flat_rotation.apply({0., r, h});
        EXPECT_TRUE(CompareVectors(flat_dut.W_of_prh(p, r, h), geo_position, kVeryExact));
      }
    }
  }

  // Checks for linear elevated curve.
  const double kElevationSlope = 15. / (kRadius * kDTheta);
  const double kElevationOffset = 10. / (kRadius * kDTheta);
  const CubicPolynomial linear_elevation(kElevationOffset, kElevationSlope, 0., 0.);
  const ArcRoadCurve elevated_dut(kCenter, kRadius, kTheta0, kDTheta, linear_elevation, zp, kLinearTolerance,
                                  kScaleLength, kComputationPolicy);
  // Computes the rotation along the RoadCurve.
  const math::Vector3 z_vector(0., 0., kRadius * kDTheta);
  const double kZeroRoll{0.};
  const double kLinearElevatedPitch = -std::atan(linear_elevation.f_dot_p(0.));
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        const Rot3 elevated_rotation(kZeroRoll, kLinearElevatedPitch, flat_dut.heading_of_p(p));
        const double angle = kTheta0 + kDTheta * p;
        const math::Vector3 geo_position = kGeoCenter + kRadius * math::Vector3(std::cos(angle), std::sin(angle), 0.) +
                                           linear_elevation.f_p(p) * z_vector + elevated_rotation.apply({0., r, h});
        EXPECT_TRUE(CompareVectors(elevated_dut.W_of_prh(p, r, h), geo_position, kVeryExact));
      }
    }
  }

  // Checks for a curve with constant non zero superelevation.
  const double kSuperelevationOffset = M_PI / 4.;
  const CubicPolynomial constant_offset_superelevation(kSuperelevationOffset / (kRadius * kDTheta), 0., 0., 0.);
  const ArcRoadCurve superelevated_dut(kCenter, kRadius, kTheta0, kDTheta, zp, constant_offset_superelevation,
                                       kLinearTolerance, kScaleLength, kComputationPolicy);
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        const Rot3 superelevated_rotation(kSuperelevationOffset, 0., flat_dut.heading_of_p(p));
        const double angle = kTheta0 + kDTheta * p;
        const math::Vector3 geo_position = kGeoCenter + kRadius * math::Vector3(std::cos(angle), std::sin(angle), 0.) +
                                           superelevated_rotation.apply({0., r, h});
        EXPECT_TRUE(CompareVectors(superelevated_dut.W_of_prh(p, r, h), geo_position, kVeryExact));
      }
    }
  }
}

// Checks world function derivative evaluation at different values of [p, r, h].
TEST_F(MultilaneArcRoadCurveTest, WorldFunctionDerivative) {
  const std::vector<double> r_vector{-5., 0., 5.};
  const std::vector<double> p_vector{0.0, 0.1, 0.2, 0.5, 0.7, 0.99};
  const std::vector<double> h_vector{-5., 0., 5.};

  // Numerical evaluation of the world function derivative, both in code and
  // in external software (e.g. Octave), shows a lowest discrepancy with the
  // analytical solution that's larger than the `kVeryExact` tolerance being
  // used throughout these tests and heavily dependent on the chosen
  // differential. The choices for these quantities below reflect this fact.
  const double kQuiteExact = 1e-11;
  const double kDifferential = 1e-3;
  // Numerically evaluates the derivative of a road curve world function
  // with respect to p at [p, r, h] with a five-point stencil.
  auto numeric_w_prime_of_prh = [kDifferential](const RoadCurve& dut, double p, double r, double h) -> math::Vector3 {
    const math::Vector3 dw = -1. * dut.W_of_prh(p + 2. * kDifferential, r, h) +
                             8. * dut.W_of_prh(p + kDifferential, r, h) - 8. * dut.W_of_prh(p - kDifferential, r, h) +
                             dut.W_of_prh(p - 2. * kDifferential, r, h);
    return dw / (12. * kDifferential);
  };

  // Checks for a flat curve.
  const ArcRoadCurve flat_dut(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kScaleLength,
                              kComputationPolicy);
  const math::Vector3 kGeoCenter(kCenter.x(), kCenter.y(), 0.);
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        // Computes the rotation along the RoadCurve at [p, r, h].
        const Rot3 rotation = flat_dut.Rabg_of_p(p);
        const double g_prime = flat_dut.elevation().f_dot_p(p);
        const math::Vector3 w_prime = flat_dut.W_prime_of_prh(p, r, h, rotation, g_prime);
        const math::Vector3 numeric_w_prime = numeric_w_prime_of_prh(flat_dut, p, r, h);
        EXPECT_TRUE(CompareVectors(w_prime, numeric_w_prime, kQuiteExact));
        const math::Vector3 s_hat = flat_dut.s_hat_of_prh(p, r, h, rotation, g_prime);
        EXPECT_TRUE(CompareVectors(w_prime.normalized(), s_hat, kVeryExact));
      }
    }
  }

  // Checks for linear elevated curve.
  const double kElevationSlope = 15. / (kRadius * kDTheta);
  const double kElevationOffset = 10. / (kRadius * kDTheta);
  const CubicPolynomial linear_elevation(kElevationOffset, kElevationSlope, 0., 0.);
  const ArcRoadCurve elevated_dut(kCenter, kRadius, kTheta0, kDTheta, linear_elevation, zp, kLinearTolerance,
                                  kScaleLength, kComputationPolicy);
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        // Computes the rotation along the RoadCurve at [p, r, h].
        const Rot3 rotation = elevated_dut.Rabg_of_p(p);
        const double g_prime = elevated_dut.elevation().f_dot_p(p);
        const math::Vector3 w_prime = elevated_dut.W_prime_of_prh(p, r, h, rotation, g_prime);
        const math::Vector3 numeric_w_prime = numeric_w_prime_of_prh(elevated_dut, p, r, h);
        EXPECT_TRUE(CompareVectors(w_prime, numeric_w_prime, kQuiteExact));
        const math::Vector3 s_hat = elevated_dut.s_hat_of_prh(p, r, h, rotation, g_prime);
        EXPECT_TRUE(CompareVectors(w_prime.normalized(), s_hat, kVeryExact));
      }
    }
  }

  // Checks for a curve with constant non zero superelevation.
  const double kSuperelevationOffset = M_PI / 4.;
  const CubicPolynomial constant_offset_superelevation(kSuperelevationOffset / (kRadius * kDTheta), 0., 0., 0.);
  const ArcRoadCurve superelevated_dut(kCenter, kRadius, kTheta0, kDTheta, zp, constant_offset_superelevation,
                                       kLinearTolerance, kScaleLength, kComputationPolicy);
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        // Computes the rotation along the RoadCurve at [p, r, h].
        const Rot3 rotation = superelevated_dut.Rabg_of_p(p);
        const double g_prime = superelevated_dut.elevation().f_dot_p(p);
        const math::Vector3 w_prime = superelevated_dut.W_prime_of_prh(p, r, h, rotation, g_prime);
        const math::Vector3 numeric_w_prime = numeric_w_prime_of_prh(superelevated_dut, p, r, h);
        EXPECT_TRUE(CompareVectors(w_prime, numeric_w_prime, kQuiteExact));
        const math::Vector3 s_hat = superelevated_dut.s_hat_of_prh(p, r, h, rotation, g_prime);
        EXPECT_TRUE(CompareVectors(w_prime.normalized(), s_hat, kVeryExact));
      }
    }
  }
}

// Checks reference curve rotation for different values of p. To compute the
// values being used below, the same curve functions were composed and
// numerically evaluated using Octave.
TEST_F(MultilaneArcRoadCurveTest, ReferenceCurveRotation) {
  // Wraps angles in [-π, π) range.
  auto wrap = [](double theta) {
    double theta_new = std::fmod(theta + M_PI, 2. * M_PI);
    if (theta_new < 0.) theta_new += 2. * M_PI;
    return theta_new - M_PI;
  };

  const double kZeroRoll{0.};
  const double kZeroPitch{0.};
  // Checks for a flat curve.
  {
    const ArcRoadCurve flat_dut(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kScaleLength,
                                kComputationPolicy);

    // Computes the rotation matrix and r versor at different p values and
    // checks versor direction and pitch and yaw angles which are not constants.
    Rot3 rotation = flat_dut.Rabg_of_p(0.);
    EXPECT_NEAR(rotation.roll(), kZeroRoll, kVeryExact);
    EXPECT_NEAR(rotation.pitch(), kZeroPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), 3. * M_PI / 4., kVeryExact);
    math::Vector3 r_hat = flat_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(CompareVectors(r_hat, math::Vector3(-0.707106781186548, -0.707106781186548, 0.), kVeryExact));

    rotation = flat_dut.Rabg_of_p(0.5);
    EXPECT_NEAR(rotation.roll(), kZeroRoll, kVeryExact);
    EXPECT_NEAR(rotation.pitch(), kZeroPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), -M_PI, kVeryExact);
    r_hat = flat_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(CompareVectors(r_hat, math::Vector3(0., -1., 0.), kVeryExact));

    rotation = flat_dut.Rabg_of_p(0.75);
    EXPECT_NEAR(rotation.roll(), kZeroRoll, kVeryExact);
    EXPECT_NEAR(rotation.pitch(), kZeroPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), -7. * M_PI / 8., kVeryExact);
    r_hat = flat_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(CompareVectors(r_hat, math::Vector3(0.382683432365090, -0.923879532511287, 0.), kVeryExact));

    rotation = flat_dut.Rabg_of_p(1.);
    EXPECT_NEAR(rotation.roll(), kZeroRoll, kVeryExact);
    EXPECT_NEAR(rotation.pitch(), kZeroPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), -3. * M_PI / 4., kVeryExact);
    r_hat = flat_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(CompareVectors(r_hat, math::Vector3(0.707106781186548, -0.707106781186548, 0.), kVeryExact));
  }

  // Checks for a linearly elevated curve.
  {
    const double kElevationSlope = 15. / (kRadius * kDTheta);
    const double kElevationOffset = 10. / (kRadius * kDTheta);
    const double kElevationPitch = -std::atan(kElevationSlope);
    const CubicPolynomial linear_elevation(kElevationOffset, kElevationSlope, 0., 0.);
    const ArcRoadCurve elevated_dut(kCenter, kRadius, kTheta0, kDTheta, linear_elevation, zp, kLinearTolerance,
                                    kScaleLength, kComputationPolicy);

    // Computes the rotation matrix and r versor at different p values and
    // checks versor direction and pitch and yaw angles which are not constants.
    Rot3 rotation = elevated_dut.Rabg_of_p(0.);
    EXPECT_NEAR(rotation.roll(), kZeroRoll, kVeryExact);
    EXPECT_NEAR(wrap(rotation.pitch()), kElevationPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), 3. * M_PI / 4., kVeryExact);
    math::Vector3 r_hat = elevated_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(CompareVectors(r_hat, math::Vector3(-0.707106781186548, -0.707106781186548, 0.), kVeryExact));

    rotation = elevated_dut.Rabg_of_p(0.5);
    EXPECT_NEAR(rotation.roll(), kZeroRoll, kVeryExact);
    EXPECT_NEAR(wrap(rotation.pitch()), kElevationPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), -M_PI, kVeryExact);
    r_hat = elevated_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(CompareVectors(r_hat, math::Vector3(0., -1., 0.), kVeryExact));

    rotation = elevated_dut.Rabg_of_p(0.75);
    EXPECT_NEAR(rotation.roll(), kZeroRoll, kVeryExact);
    EXPECT_NEAR(wrap(rotation.pitch()), kElevationPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), -7. * M_PI / 8., kVeryExact);
    r_hat = elevated_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(CompareVectors(r_hat, math::Vector3(0.382683432365090, -0.923879532511287, 0.), kVeryExact));

    rotation = elevated_dut.Rabg_of_p(1.);
    EXPECT_NEAR(rotation.roll(), kZeroRoll, kVeryExact);
    EXPECT_NEAR(wrap(rotation.pitch()), kElevationPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), -3. * M_PI / 4., kVeryExact);
    r_hat = elevated_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(CompareVectors(r_hat, math::Vector3(0.707106781186548, -0.707106781186548, 0.), kVeryExact));
  }

  // Checks for a curve with constant non zero superelevation.
  {
    const double kSuperelevationOffset = M_PI / 3.;
    const CubicPolynomial constant_offset_superelevation(kSuperelevationOffset / (kRadius * kDTheta), 0., 0., 0.);
    const ArcRoadCurve superelevated_dut(kCenter, kRadius, kTheta0, kDTheta, zp, constant_offset_superelevation,
                                         kLinearTolerance, kScaleLength, kComputationPolicy);

    // Computes the rotation matrix and r versor at different p values and
    // checks versor direction and pitch and yaw angles which are not constants.
    Rot3 rotation = superelevated_dut.Rabg_of_p(0.);
    EXPECT_NEAR(wrap(rotation.roll()), kSuperelevationOffset, kVeryExact);
    EXPECT_NEAR(wrap(rotation.pitch()), kZeroPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), 3. * M_PI / 4., kVeryExact);
    math::Vector3 r_hat = superelevated_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(
        CompareVectors(r_hat, math::Vector3(-0.353553390593274, -0.353553390593274, 0.866025403784439), kVeryExact));

    rotation = superelevated_dut.Rabg_of_p(0.5);
    EXPECT_NEAR(rotation.roll(), kSuperelevationOffset, kVeryExact);
    EXPECT_NEAR(wrap(rotation.pitch()), kZeroPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), -M_PI, kVeryExact);
    r_hat = superelevated_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(CompareVectors(r_hat, math::Vector3(0.0, -0.5, 0.866025403784439), kVeryExact));

    rotation = superelevated_dut.Rabg_of_p(0.75);
    EXPECT_NEAR(rotation.roll(), kSuperelevationOffset, kVeryExact);
    EXPECT_NEAR(wrap(rotation.pitch()), kZeroPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), -7. * M_PI / 8., kVeryExact);
    r_hat = superelevated_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(
        CompareVectors(r_hat, math::Vector3(0.191341716182545, -0.461939766255643, 0.866025403784439), kVeryExact));

    rotation = superelevated_dut.Rabg_of_p(1.);
    EXPECT_NEAR(rotation.roll(), kSuperelevationOffset, kVeryExact);
    EXPECT_NEAR(wrap(rotation.pitch()), kZeroPitch, kVeryExact);
    EXPECT_NEAR(wrap(rotation.yaw()), -3. * M_PI / 4., kVeryExact);
    r_hat = superelevated_dut.r_hat_of_Rabg(rotation);
    EXPECT_TRUE(
        CompareVectors(r_hat, math::Vector3(0.353553390593274, -0.353553390593274, 0.866025403784439), kVeryExact));
  }
}

// Checks orientation for different values of [p, r, h].
TEST_F(MultilaneArcRoadCurveTest, Orientation) {
  const std::vector<double> p_vector{0., 0.1, 0.2, 0.5, 0.7, 1.0};
  const std::vector<double> r_vector{-5., 0., 5.};
  const std::vector<double> h_vector{-5., 0., 5.};

  // Wraps angles in [-π, π) range.
  auto wrap = [](double theta) {
    double theta_new = std::fmod(theta + M_PI, 2. * M_PI);
    if (theta_new < 0.) theta_new += 2. * M_PI;
    return theta_new - M_PI;
  };

  // Checks for a flat curve.
  const ArcRoadCurve flat_dut(kCenter, kRadius, kTheta0, kDTheta, zp, zp, kLinearTolerance, kScaleLength,
                              kComputationPolicy);
  const double kZeroRoll{0.};
  const double kZeroPitch{0.};
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        const Rot3 rotation = flat_dut.Orientation(p, r, h);
        EXPECT_NEAR(rotation.roll(), kZeroRoll, kVeryExact);
        EXPECT_NEAR(rotation.pitch(), kZeroPitch, kVeryExact);
        EXPECT_NEAR(wrap(rotation.yaw()), wrap(kTheta0 + kDTheta * p + M_PI / 2.0), kVeryExact);
      }
    }
  }

  // Checks for a linearly elevated curve.
  const double kElevationSlope = 15. / (kRadius * kDTheta);
  const double kElevationOffset = 10. / (kRadius * kDTheta);
  const CubicPolynomial linear_elevation(kElevationOffset, kElevationSlope, 0., 0.);
  const ArcRoadCurve elevated_dut(kCenter, kRadius, kTheta0, kDTheta, linear_elevation, zp, kLinearTolerance,
                                  kScaleLength, kComputationPolicy);
  // Checks that the roll angle remains zero for all the points.
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        EXPECT_NEAR(flat_dut.Orientation(p, r, h).roll(), kZeroRoll, kVeryExact);
      }
    }
  }
  // Computes the rotation matrix at different [p, r, h] points and checks
  // pitch and yaw angles which are not constants.
  Rot3 elevated_rotation = elevated_dut.Orientation(0., -5., -5.);
  EXPECT_NEAR(wrap(elevated_rotation.pitch()), -0.55527957697788, kVeryExact);
  EXPECT_NEAR(wrap(elevated_rotation.yaw()), 2.5824595079564, kVeryExact);

  elevated_rotation = elevated_dut.Orientation(0., 0., 0.);
  EXPECT_NEAR(wrap(elevated_rotation.pitch()), -0.76234753416487, kVeryExact);
  EXPECT_NEAR(wrap(elevated_rotation.yaw()), 2.3561944901923, kVeryExact);

  elevated_rotation = elevated_dut.Orientation(0.5, 5., 5.);
  EXPECT_NEAR(wrap(elevated_rotation.pitch()), -1.0040908130143, kVeryExact);
  EXPECT_NEAR(wrap(elevated_rotation.yaw()), 2.5371890112688, kVeryExact);

  elevated_rotation = elevated_dut.Orientation(1., -5., 5.);
  EXPECT_NEAR(wrap(elevated_rotation.pitch()), -0.55527957697788, kVeryExact);
  EXPECT_NEAR(wrap(elevated_rotation.yaw()), -2.5824595079564, kVeryExact);

  // Checks for a curve with constant non zero superelevation.
  const double kSuperelevationOffset = M_PI / 3.;
  const CubicPolynomial constant_offset_superelevation(kSuperelevationOffset / (kRadius * kDTheta), 0., 0., 0.);
  const ArcRoadCurve superelevated_dut(kCenter, kRadius, kTheta0, kDTheta, zp, constant_offset_superelevation,
                                       kLinearTolerance, kScaleLength, kComputationPolicy);
  for (double p : p_vector) {
    for (double r : r_vector) {
      for (double h : h_vector) {
        const Rot3 rotation = superelevated_dut.Orientation(p, r, h);
        EXPECT_NEAR(wrap(rotation.pitch()), kZeroPitch, kVeryExact);
        EXPECT_NEAR(wrap(rotation.roll()), kSuperelevationOffset, kVeryExact);
        EXPECT_NEAR(wrap(rotation.yaw()), wrap(kTheta0 + kDTheta * p + M_PI / 2.0), kVeryExact);
      }
    }
  }
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
