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
#pragma once

#include <cmath>
#include <limits>
#include <utility>

#include <maliput/api/lane_data.h>
#include <maliput/common/maliput_abort.h>
#include <maliput/common/maliput_copyable.h>
#include <maliput/common/maliput_unused.h>
#include <maliput/math/vector.h>

#include "maliput_multilane/computation_policy.h"
#include "maliput_multilane/road_curve.h"

namespace maliput {
namespace multilane {

/// RoadCurve specification for a reference curve that describes a line.
class LineRoadCurve : public RoadCurve {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(LineRoadCurve)

  /// Constructor. Computes a line from @p xy0 as the initial point of the line
  /// and @p dxy as the difference vector that connects the @p xy0 with the end
  /// point of the reference curve.
  /// @param xy0 A 2D vector that represents the first point of the lane.
  /// @param dxy A 2D difference vector between the last point and @p xy0.
  /// @param elevation CubicPolynomial object that represents the elevation
  /// polynomial. See RoadCurve class constructor for more details.
  /// @param superelevation CubicPolynomial object that represents the
  /// superelevation polynomial. See RoadCurve class constructor for more
  /// details.
  /// @param linear_tolerance The linear tolerance, in meters, for all
  /// computations. See RoadCurve class constructor for more details.
  /// @param scale_length The minimum spatial period of variation in the curve,
  /// in meters. See RoadCurve class constructor for more details.
  /// @param computation_policy Policy to guide all computations. If geared
  /// towards speed, computations will make use of analytical expressions even
  /// if not actually correct for the curve as specified.
  /// @throws maliput::common::assertion_error if @p linear_tolerance is not a
  /// positive number.
  /// @throws maliput::common::assertion_error if @p scale_length is not a
  /// positive number.
  explicit LineRoadCurve(const math::Vector2& xy0, const math::Vector2& dxy, const CubicPolynomial& elevation,
                         const CubicPolynomial& superelevation, double linear_tolerance, double scale_length,
                         ComputationPolicy computation_policy)
      : RoadCurve(linear_tolerance, scale_length, elevation, superelevation, computation_policy),
        p0_(xy0),
        dp_(dxy),
        heading_(std::atan2(dxy.y(), dxy.x())) {
    MALIPUT_DEMAND(dxy.norm() > kMinimumNorm);
  }

  ~LineRoadCurve() override = default;

  math::Vector2 xy_of_p(double p) const override { return p0_ + p * dp_; }

  math::Vector2 xy_dot_of_p(double p) const override {
    maliput::common::unused(p);
    return dp_;
  }

  double heading_of_p(double p) const override {
    maliput::common::unused(p);
    return heading_;
  }

  double heading_dot_of_p(double p) const override {
    maliput::common::unused(p);
    return 0.;
  }

  double l_max() const override { return dp_.norm(); }

  math::Vector3 ToCurveFrame(const math::Vector3& geo_coordinate, double r_min, double r_max,
                             const api::HBounds& height_bounds) const override;

  bool IsValid(double r_min, double r_max, const api::HBounds& height_bounds) const override {
    maliput::common::unused(r_min);
    maliput::common::unused(r_max);
    maliput::common::unused(height_bounds);
    return true;
  }

 private:
  double FastCalcPFromS(double s, double r) const override;

  double FastCalcSFromP(double p, double r) const override;

  double CalcMinimumRadiusAtOffset(double r) const override {
    maliput::common::unused(r);
    return std::numeric_limits<double>::infinity();
  }

  // The first point in world coordinates over the z=0 plane of the reference
  // curve.
  const math::Vector2 p0_{};
  // The difference vector that joins the end point of the reference curve with
  // the first one, p0_.
  const math::Vector2 dp_{};
  // The constant angle deviation of dp_ with respect to the x axis of the world
  // frame.
  const double heading_{};
  // The minimum dp_ norm to avoid having issues when computing heading_.
  static const double kMinimumNorm;
};

}  // namespace multilane
}  // namespace maliput
