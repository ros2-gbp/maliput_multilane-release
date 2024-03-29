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
#include "maliput_multilane/connection.h"
/* clang-format on */

#include <cmath>
#include <ostream>

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/maliput_math_compare.h>

#include "maliput_multilane/arc_road_curve.h"
#include "maliput_multilane/cubic_polynomial.h"
#include "maliput_multilane/line_road_curve.h"
#include "maliput_multilane/make_road_curve_for_connection.h"
#include "maliput_multilane_test_utilities/multilane_types_compare.h"

namespace maliput {
namespace multilane {
namespace test {

using maliput::math::test::CompareVectors;

// Compares equality within @p tolerance of @p cubic1 and @p cubic2
// coefficients.
// @param cubic1 A CubicPolynomial object to compare.
// @param cubic2 A CubicPolynomial object to compare.
// @param tolerance An allowable absolute linear deviation for each coefficient.
// @return ::testing::AssertionFailure() When any coefficient of
// CubicPolynomial objects are different.
// @return ::testing::AssertionSuccess() When all coefficients of
// CubicPolynomial objects are equal.
::testing::AssertionResult IsCubicPolynomialClose(const CubicPolynomial& cubic1, const CubicPolynomial& cubic2,
                                                  double tolerance) {
  bool fails = false;
  std::string error_message{};
  const std::vector<std::string> coefficient_strs{"a", "b", "c", "d"};
  const std::vector<double> coefficients1{cubic1.a(), cubic1.b(), cubic1.c(), cubic1.d()};
  const std::vector<double> coefficients2{cubic2.a(), cubic2.b(), cubic2.c(), cubic2.d()};

  for (int i = 0; i < 4; ++i) {
    const double delta = std::abs(coefficients1[i] - coefficients2[i]);
    if (delta > tolerance) {
      fails = true;
      error_message += fmt::format(
          "Cubic polynomials are different at {0} coefficient. "
          "cubic1.{0}(): {1} vs. cubic2.{0}(): {2}, diff = {3}, "
          "tolerance = {4}\n",
          coefficient_strs[i], coefficients1[i], coefficients2[i], delta, tolerance);
    }
  }
  if (fails) {
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << fmt::format(
             "cubic1 =\n{}\nis approximately equal to cubic2 =\n{}"
             "\ntolerance = {}",
             cubic1, cubic2, tolerance);
}

}  // namespace test

namespace {

using maliput::math::test::CompareVectors;

// EndpointXy checks.
GTEST_TEST(EndpointXyTest, DefaultConstructor) {
  const EndpointXy dut{};
  EXPECT_EQ(dut.x(), 0.);
  EXPECT_EQ(dut.y(), 0.);
  EXPECT_EQ(dut.heading(), 0.);
}

GTEST_TEST(EndpointXyTest, ParametrizedConstructor) {
  const EndpointXy dut{1., 2., M_PI / 4.};
  EXPECT_EQ(dut.x(), 1.);
  EXPECT_EQ(dut.y(), 2.);
  EXPECT_EQ(dut.heading(), M_PI / 4.);
}

GTEST_TEST(EndpointXyTest, Reverse) {
  const EndpointXy dut{1., 2., M_PI / 7.123456};
  constexpr double kEndpointLinearTolerance{1e-15};
  constexpr double kEndpointAngularTolerance{1e-15};
  EXPECT_TRUE(test::IsEndpointXyClose(dut.reverse(), {1., 2., -M_PI * (1. - 1. / 7.123456)}, kEndpointLinearTolerance,
                                      kEndpointAngularTolerance));
}

// EndpointZ checks.
GTEST_TEST(EndpointZTest, DefaultConstructor) {
  const EndpointZ dut{};
  EXPECT_EQ(dut.z(), 0.);
  EXPECT_EQ(dut.z_dot(), 0.);
  EXPECT_EQ(dut.theta(), 0.);
  EXPECT_FALSE(dut.theta_dot().has_value());
}

GTEST_TEST(EndpointZTest, ParametrizedConstructors) {
  const EndpointZ dut_without_theta_dot{1., 2., M_PI / 4., {}};
  EXPECT_EQ(dut_without_theta_dot.z(), 1.);
  EXPECT_EQ(dut_without_theta_dot.z_dot(), 2.);
  EXPECT_EQ(dut_without_theta_dot.theta(), M_PI / 4.);
  EXPECT_FALSE(dut_without_theta_dot.theta_dot().has_value());

  const EndpointZ dut_with_theta_dot{1., 2., M_PI / 4., M_PI / 2.};
  EXPECT_EQ(dut_with_theta_dot.z(), 1.);
  EXPECT_EQ(dut_with_theta_dot.z_dot(), 2.);
  EXPECT_EQ(dut_with_theta_dot.theta(), M_PI / 4.);
  EXPECT_TRUE(dut_with_theta_dot.theta_dot().has_value());
  EXPECT_EQ(*dut_with_theta_dot.theta_dot(), M_PI / 2.);
}

GTEST_TEST(EndpointZTest, Reverse) {
  constexpr double kZeroTolerance{0.};

  const EndpointZ dut_without_theta_dot{1., 2., M_PI / 4., {}};
  EXPECT_TRUE(test::IsEndpointZClose(dut_without_theta_dot.reverse(), {1., -2., -M_PI / 4., {}}, kZeroTolerance,
                                     kZeroTolerance));

  const EndpointZ dut_with_theta_dot{1., 2., M_PI / 4., M_PI / 2.};
  EXPECT_TRUE(test::IsEndpointZClose(dut_with_theta_dot.reverse(), {1., -2., -M_PI / 4., M_PI / 2.}, kZeroTolerance,
                                     kZeroTolerance));
}

// LineOffset check.
GTEST_TEST(LineOffsetTest, ParametrizedConstructor) {
  const LineOffset dut{123.456};
  EXPECT_EQ(dut.length(), 123.456);
}

// ArcOffset check.
GTEST_TEST(ArcOffsetTest, ParametrizedConstructor) {
  const ArcOffset dut{1., M_PI / 4.};
  EXPECT_EQ(dut.radius(), 1.);
  EXPECT_EQ(dut.d_theta(), M_PI / 4.);
}

// Connection checks.
class MultilaneConnectionTest : public ::testing::Test {
 protected:
  const double kR0{2.};
  const int kNumLanes{3};
  const double kLeftShoulder{1.};
  const double kRightShoulder{1.5};
  const double kLaneWidth{2.};
  const EndpointZ kLowFlatZ{0., 0., 0., 0.};
  const double kHeading{-M_PI / 4.};
  const EndpointXy kStartXy{20., 30., kHeading};
  const Endpoint kStartEndpoint{kStartXy, kLowFlatZ};
  const double kZeroTolerance{0.};
  const double kEndpointLinearTolerance{1e-12};
  const double kEndpointAngularTolerance{1e-12};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.0};
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};
  const double kVeryExact{1e-12};
};

TEST_F(MultilaneConnectionTest, ArcAccessors) {
  const std::string kId{"arc_connection"};
  const double kRadius{10. * std::sqrt(2.)};
  const double kDTheta{M_PI / 2.};
  const ArcOffset kArcOffset(kRadius, kDTheta);
  const Endpoint kEndEndpoint{{40., 30., kHeading + kDTheta}, kLowFlatZ};

  const Connection dut(kId, kStartEndpoint, kLowFlatZ, kNumLanes, kR0, kLaneWidth, kLeftShoulder, kRightShoulder,
                       kArcOffset, kLinearTolerance, kScaleLength, kComputationPolicy);

  EXPECT_EQ(dut.type(), Connection::Type::kArc);
  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.num_lanes(), kNumLanes);
  EXPECT_EQ(dut.r0(), kR0);
  EXPECT_EQ(dut.lane_width(), kLaneWidth);
  EXPECT_EQ(dut.left_shoulder(), kLeftShoulder);
  EXPECT_EQ(dut.right_shoulder(), kRightShoulder);
  EXPECT_EQ(dut.radius(), kRadius);
  EXPECT_EQ(dut.d_theta(), kDTheta);
  EXPECT_EQ(dut.r_min(), kR0 - kLaneWidth / 2. - kRightShoulder);
  EXPECT_EQ(dut.r_max(), kR0 + kLaneWidth * (static_cast<double>(kNumLanes - 1) + .5) + kLeftShoulder);
  EXPECT_TRUE(test::IsEndpointClose(dut.start(), kStartEndpoint, kZeroTolerance, kZeroTolerance));
  EXPECT_TRUE(test::IsEndpointClose(dut.end(), kEndEndpoint, kZeroTolerance, kZeroTolerance));
  EXPECT_EQ(dut.lane_offset(0), kR0);
  EXPECT_EQ(dut.lane_offset(1), kR0 + kLaneWidth);
  EXPECT_EQ(dut.lane_offset(2), kR0 + 2. * kLaneWidth);
  EXPECT_EQ(dut.computation_policy(), kComputationPolicy);
  EXPECT_EQ(dut.linear_tolerance(), kLinearTolerance);
  EXPECT_EQ(dut.scale_length(), kScaleLength);
}

TEST_F(MultilaneConnectionTest, LineAccessors) {
  const std::string kId{"line_connection"};
  const Endpoint kEndEndpoint{{50., 0., kHeading}, kLowFlatZ};

  const double kLineLength{30. * std::sqrt(2.)};
  const LineOffset kLineOffset{kLineLength};
  const Connection dut(kId, kStartEndpoint, kLowFlatZ, kNumLanes, kR0, kLaneWidth, kLeftShoulder, kRightShoulder,
                       kLineOffset, kLinearTolerance, kScaleLength, kComputationPolicy);
  EXPECT_EQ(dut.type(), Connection::Type::kLine);
  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.num_lanes(), kNumLanes);
  EXPECT_EQ(dut.r0(), kR0);
  EXPECT_EQ(dut.lane_width(), kLaneWidth);
  EXPECT_EQ(dut.left_shoulder(), kLeftShoulder);
  EXPECT_EQ(dut.right_shoulder(), kRightShoulder);
  EXPECT_EQ(dut.line_length(), kLineLength);
  EXPECT_EQ(dut.r_min(), kR0 - kLaneWidth / 2. - kRightShoulder);
  EXPECT_EQ(dut.r_max(), kR0 + kLaneWidth * (static_cast<double>(kNumLanes - 1) + .5) + kLeftShoulder);
  EXPECT_TRUE(test::IsEndpointClose(dut.start(), kStartEndpoint, kZeroTolerance, kZeroTolerance));
  EXPECT_TRUE(test::IsEndpointClose(dut.end(), kEndEndpoint, kZeroTolerance, kZeroTolerance));
  EXPECT_EQ(dut.lane_offset(0), kR0);
  EXPECT_EQ(dut.lane_offset(1), kR0 + kLaneWidth);
  EXPECT_EQ(dut.lane_offset(2), kR0 + 2. * kLaneWidth);
  EXPECT_EQ(dut.computation_policy(), kComputationPolicy);
  EXPECT_EQ(dut.linear_tolerance(), kLinearTolerance);
  EXPECT_EQ(dut.scale_length(), kScaleLength);
}

// Checks RoadCurve creation.
//
// Literals for elevation and superelevation polynomials below have been derived
// in Octave running the following code snippet. Variables with '0' suffix refer
// to start EndpointZ and with '1' suffix refer to end EndpointZ. Replace 'y'
// variables by 'z' related ones and by 'theta' to compute elevation and
// superelevation polynomial coefficients respectively.
//
// % Sets the value of the planar length.
// d_x = 10. * sqrt(2.) * pi / 2.;
// a = y_0 / d_x
// b = y_dot_0 / d_x
// c = 3 * (y_1 - y_0) / d_x - 2 * y_dot_0 - y_dot_1
// d = y_dot_0 + y_dot_1 - 2 * (y_1 - y_0)
TEST_F(MultilaneConnectionTest, ArcRoadCurveValidation) {
  const std::string kId{"arc_connection"};
  const double kRadius{10. * std::sqrt(2.)};
  const double kDTheta{M_PI / 2.};
  const ArcOffset kArcOffset(kRadius, kDTheta);
  const Endpoint kEndEndpoint{{40., 30., kHeading + kDTheta}, kLowFlatZ};

  const Connection flat_dut(kId, kStartEndpoint, kLowFlatZ, kNumLanes, kR0, kLaneWidth, kLeftShoulder, kRightShoulder,
                            kArcOffset, kLinearTolerance, kScaleLength, kComputationPolicy);
  std::unique_ptr<RoadCurve> road_curve = MakeRoadCurveFor(flat_dut);
  EXPECT_NE(dynamic_cast<ArcRoadCurve*>(road_curve.get()), nullptr);
  // Checks that the road curve starts and ends at given endpoints.
  const math::Vector3 flat_origin = road_curve->W_of_prh(0., 0., 0.);
  EXPECT_TRUE(
      CompareVectors(math::Vector3(flat_dut.start().xy().x(), flat_dut.start().xy().y(), flat_dut.start().z().z()),
                     flat_origin, kZeroTolerance));
  EXPECT_TRUE(CompareVectors(math::Vector3(kStartEndpoint.xy().x(), kStartEndpoint.xy().y(), kStartEndpoint.z().z()),
                             flat_origin, kZeroTolerance));
  const math::Vector3 flat_end = road_curve->W_of_prh(1., 0., 0.);
  EXPECT_TRUE(CompareVectors(math::Vector3(flat_dut.end().xy().x(), flat_dut.end().xy().y(), flat_dut.end().z().z()),
                             flat_end, kVeryExact));
  EXPECT_TRUE(CompareVectors(math::Vector3(kEndEndpoint.xy().x(), kEndEndpoint.xy().y(), kEndEndpoint.z().z()),
                             flat_end, kVeryExact));
  // Checks that elevation and superelevation polynomials are correctly built
  // for the trivial case of a flat dut.
  EXPECT_TRUE(test::IsCubicPolynomialClose(road_curve->elevation(), CubicPolynomial(), kZeroTolerance));
  EXPECT_TRUE(test::IsCubicPolynomialClose(road_curve->superelevation(), CubicPolynomial(), kZeroTolerance));

  // Creates a new complex dut with cubic elevation and superelevation.
  const Endpoint kEndElevatedEndpoint{{40., 30., kHeading + kDTheta}, {5., 1., M_PI / 6., 1.}};
  const Connection complex_dut(kId, kStartEndpoint, kEndElevatedEndpoint.z(), kNumLanes, kR0, kLaneWidth, kLeftShoulder,
                               kRightShoulder, kArcOffset, kLinearTolerance, kScaleLength, kComputationPolicy);
  std::unique_ptr<RoadCurve> complex_road_curve = MakeRoadCurveFor(complex_dut);
  // Checks that the road curve starts and ends at given endpoints.
  const math::Vector3 complex_origin = complex_road_curve->W_of_prh(0., 0., 0.);
  EXPECT_TRUE(CompareVectors(
      math::Vector3(complex_dut.start().xy().x(), complex_dut.start().xy().y(), complex_dut.start().z().z()),
      complex_origin, kZeroTolerance));
  EXPECT_TRUE(CompareVectors(math::Vector3(kStartEndpoint.xy().x(), kStartEndpoint.xy().y(), kStartEndpoint.z().z()),
                             complex_origin, kZeroTolerance));
  const math::Vector3 complex_end = complex_road_curve->W_of_prh(1., 0., 0.);
  EXPECT_TRUE(
      CompareVectors(math::Vector3(complex_dut.end().xy().x(), complex_dut.end().xy().y(), complex_dut.end().z().z()),
                     complex_end, kVeryExact));
  EXPECT_TRUE(CompareVectors(
      math::Vector3(kEndElevatedEndpoint.xy().x(), kEndElevatedEndpoint.xy().y(), kEndElevatedEndpoint.z().z()),
      complex_end, kVeryExact));
  EXPECT_TRUE(test::IsCubicPolynomialClose(
      complex_road_curve->elevation(), CubicPolynomial(0., 0., -0.32476276288217043, 0.549841841921447), kVeryExact));
  EXPECT_TRUE(test::IsCubicPolynomialClose(complex_road_curve->superelevation(),
                                           CubicPolynomial(0., 0., -0.9292893218813453, 0.9528595479208968),
                                           kVeryExact));
}

TEST_F(MultilaneConnectionTest, LineRoadCurveValidation) {
  const std::string kId{"line_connection"};
  const Endpoint kEndEndpoint{{50., 0., kHeading}, kLowFlatZ};
  const double kLineLength{30. * std::sqrt(2.)};
  const LineOffset kLineOffset{kLineLength};
  const Connection flat_dut(kId, kStartEndpoint, kLowFlatZ, kNumLanes, kR0, kLaneWidth, kLeftShoulder, kRightShoulder,
                            kLineOffset, kLinearTolerance, kScaleLength, kComputationPolicy);
  std::unique_ptr<RoadCurve> road_curve = MakeRoadCurveFor(flat_dut);
  EXPECT_NE(dynamic_cast<LineRoadCurve*>(road_curve.get()), nullptr);

  // Checks that the road curve starts and ends at given endpoints.
  const math::Vector3 flat_origin = road_curve->W_of_prh(0., 0., 0.);
  EXPECT_TRUE(
      CompareVectors(math::Vector3(flat_dut.start().xy().x(), flat_dut.start().xy().y(), flat_dut.start().z().z()),
                     flat_origin, kZeroTolerance));
  EXPECT_TRUE(CompareVectors(math::Vector3(kStartEndpoint.xy().x(), kStartEndpoint.xy().y(), kStartEndpoint.z().z()),
                             flat_origin, kZeroTolerance));
  const math::Vector3 flat_end = road_curve->W_of_prh(1., 0., 0.);
  EXPECT_TRUE(CompareVectors(math::Vector3(flat_dut.end().xy().x(), flat_dut.end().xy().y(), flat_dut.end().z().z()),
                             flat_end, kVeryExact));
  EXPECT_TRUE(CompareVectors(math::Vector3(kEndEndpoint.xy().x(), kEndEndpoint.xy().y(), kEndEndpoint.z().z()),
                             flat_end, kVeryExact));
  // Checks that elevation and superelevation polynomials are correctly built
  // for the trivial case of a flat dut.
  EXPECT_TRUE(test::IsCubicPolynomialClose(road_curve->elevation(), CubicPolynomial(), kZeroTolerance));
  EXPECT_TRUE(test::IsCubicPolynomialClose(road_curve->superelevation(), CubicPolynomial(), kZeroTolerance));

  // Creates a new complex dut with cubic elevation and superelevation.
  const Endpoint kEndElevatedEndpoint{{50., 0., kHeading}, {5., 1., M_PI / 6., 1.}};
  const Connection complex_dut(kId, kStartEndpoint, kEndElevatedEndpoint.z(), kNumLanes, kR0, kLaneWidth, kLeftShoulder,
                               kRightShoulder, kLineOffset, kLinearTolerance, kScaleLength, kComputationPolicy);
  std::unique_ptr<RoadCurve> complex_road_curve = MakeRoadCurveFor(complex_dut);

  // Checks that the road curve starts and ends at given endpoints.
  const math::Vector3 complex_origin = complex_road_curve->W_of_prh(0., 0., 0.);
  EXPECT_TRUE(CompareVectors(
      math::Vector3(complex_dut.start().xy().x(), complex_dut.start().xy().y(), complex_dut.start().z().z()),
      complex_origin, kZeroTolerance));
  EXPECT_TRUE(CompareVectors(math::Vector3(kStartEndpoint.xy().x(), kStartEndpoint.xy().y(), kStartEndpoint.z().z()),
                             complex_origin, kZeroTolerance));
  const math::Vector3 complex_end = complex_road_curve->W_of_prh(1., 0., 0.);
  EXPECT_TRUE(
      CompareVectors(math::Vector3(complex_dut.end().xy().x(), complex_dut.end().xy().y(), complex_dut.end().z().z()),
                     complex_end, kVeryExact));
  EXPECT_TRUE(CompareVectors(
      math::Vector3(kEndElevatedEndpoint.xy().x(), kEndElevatedEndpoint.xy().y(), kEndElevatedEndpoint.z().z()),
      complex_end, kVeryExact));
  EXPECT_TRUE(test::IsCubicPolynomialClose(complex_road_curve->elevation(),
                                           CubicPolynomial(0., 0., -0.646446609406726, 0.764297739604484), kVeryExact));
  EXPECT_TRUE(test::IsCubicPolynomialClose(complex_road_curve->superelevation(),
                                           CubicPolynomial(0., 0., -0.962975975515347, 0.975317317010231), kVeryExact));
}

// Lane Endpoints with different EndpointZ. Those are selected to cover
// different combinations of elevation and superelevation polynomials.

// Groups test parameters.
struct EndpointZTestParameters {
  EndpointZTestParameters() = default;
  EndpointZTestParameters(const EndpointZ& _start_z, const EndpointZ& _end_z, double _r0, int _num_lanes)
      : start_z(_start_z), end_z(_end_z), r0(_r0), num_lanes(_num_lanes) {}

  EndpointZ start_z{};
  EndpointZ end_z{};
  double r0{};
  int num_lanes{};
};

// Stream insertion operator overload for EndpointZTestParameters
// instances. Necessary for gtest printouts that would otherwise fail
// at properly printing the struct's bytes (its default behavior when
// no stream insertion operator overload is present) and trigger Valgrind
// errors.
std::ostream& operator<<(std::ostream& stream, const EndpointZTestParameters& endpoint_z_test_param) {
  return stream << fmt::format(
             "EndpointZTestParameters( start_z: ({}), "
             "end_z: ({}), r0: {}, num_lanes: {})",
             endpoint_z_test_param.start_z, endpoint_z_test_param.end_z, endpoint_z_test_param.r0,
             endpoint_z_test_param.num_lanes);
}

// Groups common test constants as well as each test case parameters.
class MultilaneConnectionEndpointZTest : public ::testing::TestWithParam<EndpointZTestParameters> {
 protected:
  void SetUp() override {
    const EndpointZTestParameters parameters = this->GetParam();
    start_z = parameters.start_z;
    end_z = parameters.end_z;
    r0 = parameters.r0;
    num_lanes = parameters.num_lanes;
    start_endpoint = {kStartXy, start_z};
  }

  const double kLeftShoulder{1.};
  const double kRightShoulder{1.5};
  const double kLaneWidth{2.};
  const double kHeading{-M_PI / 4.};
  const EndpointXy kStartXy{20., 30., kHeading};
  const double kEndpointLinearTolerance{1e-12};
  const double kEndpointAngularTolerance{1e-12};
  const double kLinearTolerance{0.01};
  const double kScaleLength{1.0};
  const ComputationPolicy kComputationPolicy{ComputationPolicy::kPreferAccuracy};
  EndpointZ start_z{};
  EndpointZ end_z{};
  double r0{};
  int num_lanes{};
  Endpoint start_endpoint{};
  const double kVeryExact{1e-12};
};

TEST_P(MultilaneConnectionEndpointZTest, ArcLaneEndpoints) {
  const std::string kId{"arc_connection"};
  const double kCenterX{30.};
  const double kCenterY{40.};
  const double kRadius{10. * std::sqrt(2.)};
  const double kDTheta{M_PI / 2.};
  const ArcOffset kArcOffset(kRadius, kDTheta);
  const Connection dut(kId, start_endpoint, end_z, num_lanes, r0, kLaneWidth, kLeftShoulder, kRightShoulder, kArcOffset,
                       kLinearTolerance, kScaleLength, kComputationPolicy);
  const double kTheta0{kHeading - M_PI / 2.};

  // Wraps angles in [-π, π) range.
  auto wrap = [](double theta) {
    double theta_new = std::fmod(theta + M_PI, 2. * M_PI);
    if (theta_new < 0.) theta_new += 2. * M_PI;
    return theta_new - M_PI;
  };

  for (int i = 0; i < num_lanes; i++) {
    // Start endpoints.
    const double start_radius = kRadius - (r0 + static_cast<double>(i) * kLaneWidth) * std::cos(start_z.theta());
    const Endpoint lane_start{{kCenterX + start_radius * std::cos(kTheta0), kCenterY + start_radius * std::sin(kTheta0),
                               wrap(kTheta0 + M_PI / 2.)},
                              {start_z.z(), start_z.z_dot() * kRadius / start_radius, start_z.theta(),
                               (*start_z.theta_dot()) * kRadius / start_radius}};
    EXPECT_TRUE(
        test::IsEndpointClose(dut.LaneStart(i), lane_start, kEndpointLinearTolerance, kEndpointAngularTolerance));
    // End endpoints.
    const double end_radius = kRadius - (r0 + static_cast<double>(i) * kLaneWidth) * std::cos(end_z.theta());
    const Endpoint lane_end{
        {kCenterX + end_radius * std::cos(kTheta0 + kDTheta), kCenterY + end_radius * std::sin(kTheta0 + kDTheta),
         wrap(kTheta0 + kDTheta + M_PI / 2.)},
        {end_z.z(), end_z.z_dot() * kRadius / end_radius, end_z.theta(), (*end_z.theta_dot()) * kRadius / end_radius}};
    EXPECT_TRUE(test::IsEndpointClose(dut.LaneEnd(i), lane_end, kEndpointLinearTolerance, kEndpointAngularTolerance));
  }
}

TEST_P(MultilaneConnectionEndpointZTest, LineLaneEndpoints) {
  const std::string kId{"line_connection"};
  const double kLineLength{25. * std::sqrt(2.)};
  const LineOffset kLineOffset{kLineLength};
  const Connection dut(kId, start_endpoint, end_z, num_lanes, r0, kLaneWidth, kLeftShoulder, kRightShoulder,
                       kLineOffset, kLinearTolerance, kScaleLength, kComputationPolicy);
  const math::Vector2 kDirection{45. - kStartXy.x(), 5. - kStartXy.y()};
  const math::Vector2 kNormalDirection = math::Vector2(kDirection.y(), -kDirection.x()).normalized();

  for (int i = 0; i < num_lanes; i++) {
    // Start endpoints.
    const double offset = r0 + static_cast<double>(i) * kLaneWidth;
    const Endpoint lane_start{
        {kStartXy.x() - offset * kNormalDirection.x(), kStartXy.y() - offset * kNormalDirection.y(), kHeading},
        start_z};
    EXPECT_TRUE(
        test::IsEndpointClose(dut.LaneStart(i), lane_start, kEndpointLinearTolerance, kEndpointAngularTolerance));
    // End endpoints.
    const Endpoint lane_end{{kStartXy.x() - offset * kNormalDirection.x() + kDirection.x(),
                             kStartXy.y() - offset * kNormalDirection.y() + kDirection.y(), kHeading},
                            end_z};
    EXPECT_TRUE(test::IsEndpointClose(dut.LaneEnd(i), lane_end, kEndpointLinearTolerance, kEndpointAngularTolerance));
  }
}

INSTANTIATE_TEST_CASE_P(
    EndpointZ, MultilaneConnectionEndpointZTest,
    testing::Values(EndpointZTestParameters(EndpointZ(0., 0., 0., 0.), EndpointZ(0., 0., 0., 0.), 0., 1),
                    EndpointZTestParameters(EndpointZ(0., 0., 0., 0.), EndpointZ(0., 0., 0., 0.), 2., 3),
                    EndpointZTestParameters(EndpointZ(1., 0., 0., 0.), EndpointZ(1., 0., 0., 0.), 2., 3),
                    EndpointZTestParameters(EndpointZ(1., 1., 0., 0.), EndpointZ(5., 1., 0., 0.), 2., 3),
                    EndpointZTestParameters(EndpointZ(0., 0., M_PI / 6., 0.), EndpointZ(0., 0., M_PI / 6., 0.), 0., 1),
                    EndpointZTestParameters(EndpointZ(1., 1., M_PI / 6., 0.), EndpointZ(5., 1., M_PI / 6., 0.), 0., 1),
                    EndpointZTestParameters(EndpointZ(1., 1., M_PI / 3., 1.), EndpointZ(5., 1., M_PI / 6., 1.), 0.,
                                            1)));

// Makes a stub line-connection.
std::unique_ptr<Connection> MakeLineConnection() {
  return std::make_unique<Connection>(
      "line_connection" /* Id */, Endpoint({20., 30., 0.}, {0., 0., 0., 0.}) /* Start Endpoint */,
      EndpointZ(0., 0., 0., 0.) /* End EndpointZ */, 3 /* Number of lanes */, 0. /* kR0 */, 5. /* Lane width */,
      1. /* Left shoulder */, 1. /* Right shoulder */, LineOffset(100.), 0.01 /* Linear tolerance */,
      1. /* scale length */, ComputationPolicy::kPreferAccuracy);
}

// Makes a stub arc-connection.
std::unique_ptr<Connection> MakeArcConnection() {
  return std::make_unique<Connection>(
      "arc_connection" /* Id */, Endpoint({20., 30., 0.}, {0., 0., 0., 0.}) /* Start Endpoint */,
      EndpointZ(0., 0., 0., 0.) /* End EndpointZ */, 3 /* Number of lanes */, 0. /* kR0 */, 5. /* Lane width */,
      1. /* Left shoulder */, 1. /* Right shoulder */, ArcOffset(30., M_PI / 2.), 0.01 /* Linear tolerance */,
      1. /* scale length */, ComputationPolicy::kPreferAccuracy);
}

GTEST_TEST(GroupTest, NoConnection) {
  const std::string kId{"Empty_Group"};
  std::unique_ptr<Group> dut = GroupFactory().Make(kId);

  EXPECT_EQ(dut->id(), kId);
  EXPECT_TRUE(dut->connections().empty());
}

GTEST_TEST(GroupTest, WithConnections) {
  const std::string kId{"Populated_Group"};

  auto line_connection = MakeLineConnection();
  auto arc_connection = MakeArcConnection();

  std::unique_ptr<Group> dut = GroupFactory().Make(kId, {line_connection.get(), arc_connection.get()});

  EXPECT_EQ(dut->id(), kId);
  EXPECT_EQ(static_cast<int>(dut->connections().size()), 2);
  EXPECT_EQ(dut->connections()[0], line_connection.get());
  EXPECT_EQ(dut->connections()[1], arc_connection.get());
}

GTEST_TEST(GroupTest, AddConnections) {
  const std::string kId{"Group"};

  auto line_connection = MakeLineConnection();
  auto arc_connection = MakeArcConnection();

  std::unique_ptr<Group> dut = GroupFactory().Make(kId, {line_connection.get()});

  EXPECT_EQ(static_cast<int>(dut->connections().size()), 1);
  EXPECT_EQ(dut->connections()[0], line_connection.get());

  dut->Add(arc_connection.get());
  EXPECT_EQ(static_cast<int>(dut->connections().size()), 2);
  EXPECT_EQ(dut->connections()[0], line_connection.get());
  EXPECT_EQ(dut->connections()[1], arc_connection.get());
}

}  // namespace
}  // namespace multilane
}  // namespace maliput
