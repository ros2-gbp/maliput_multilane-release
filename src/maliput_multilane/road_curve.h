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

#include <functional>
#include <memory>
#include <utility>

#include <maliput/api/lane_data.h>
#include <maliput/common/maliput_copyable.h>
#include <maliput/drake/systems/analysis/antiderivative_function.h>
#include <maliput/drake/systems/analysis/scalar_initial_value_problem.h>
#include <maliput/math/roll_pitch_yaw.h>
#include <maliput/math/vector.h>

#include "maliput_multilane/computation_policy.h"
#include "maliput_multilane/cubic_polynomial.h"

namespace maliput {
namespace multilane {

/// An R^3 rotation parameterized by roll, pitch, yaw.
///
/// This effects a compound rotation around space-fixed x-y-z axes:
///
///   Rot3(roll, pitch, yaw) * V = RotZ(yaw) * RotY(pitch) * RotX(roll) * V
class Rot3 {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rot3)

  Rot3(double roll, double pitch, double yaw) : rpy_(roll, pitch, yaw) {}

  /// Applies the rotation to a 3-vector.
  math::Vector3 apply(const math::Vector3& in) const { return rpy_.ToMatrix() * in; }

  double yaw() const { return rpy_.yaw_angle(); }
  double pitch() const { return rpy_.pitch_angle(); }
  double roll() const { return rpy_.roll_angle(); }

 private:
  math::RollPitchYaw rpy_;
};

// TODO(maddog-tri)  Add support for Lanes with both non-zero r0 and
//                   superelevation polynomial.
/// Defines an interface for a path in a Segment object surface. The path is
/// defined by an elevation and superelevation CubicPolynomial objects and a
/// reference curve. This reference curve is a C1 function in the z=0 plane.
/// Its domain is constrained in [0;1] interval and it should map a ℝ² curve.
/// As per notation, p is the parameter of the reference curve, not necessarily
/// arc length s, and function interpolations and function derivatives as well
/// as headings and heading derivatives are expressed in world coordinates,
/// which is the same frame as api::InertialPosition.
/// By implementing this interface the road curve is defined and complete.
///
/// The geometry here revolves around an abstract "world function"
///
///    W: (p,r,h) --> (x,y,z)
///
/// which maps a `Lane`-frame position to its corresponding representation in
/// world coordinates (with the caveat that instead of the lane's native
/// longitudinal coordinate 's', the reference curve parameter 'p' is used).
///
/// W is derived from the three functions which define the lane:
///
///   G: p --> (x,y)     = the reference curve, a.k.a. xy_of_p()
///   Z: p --> z / l_max = the elevation function, a.k.a. elevation_
///   Θ: p --> θ / l_max = the superelevation function, a.k.a. superelevation_
///
/// as:
///
///   (x,y,z) = W(p,r,h) = (G(p), Z(p)) + R_αβγ*(0,r,h)
///
/// where R_αβγ is the roll/pitch/yaw rotation given by angles:
///
///   α = Θ(p)
///   β = -atan(dZ/dp) at p
///   γ = atan2(dG_y/dp, dG_x/dp) at p
///
/// (R_αβγ is essentially the orientation of the (s,r,h) `Lane`-frame
/// at a location (s,0,0) on the reference-line of the lane.  However, it
/// is *not* necessarily the correct orientation at r != 0 or h != 0.)
///
/// The W(p,r,h) "world function" is defined by the RoadCurve referenced by a
/// Lane's Segment. A Lane is also defined by a r0 lateral offset with respect
/// to the reference curve of the RoadCurve. Thus, a mapping from the local
/// (s,r,h) lane-frame of the Lane becomes:
///
/// (x,y,z) = L(s,r,h) = W(P(s, r0), r + r0, h),
///
/// where P:(s, r0) --> (p) is a (potentially non-linear) function dependent on
/// the RoadCurve's reference-curve, elevation, and superelevation functions.
class RoadCurve {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadCurve)

  virtual ~RoadCurve() = default;

  const CubicPolynomial& elevation() const { return elevation_; }

  const CubicPolynomial& superelevation() const { return superelevation_; }

  const double& linear_tolerance() const { return linear_tolerance_; }

  const double& scale_length() const { return scale_length_; }

  const ComputationPolicy& computation_policy() const { return computation_policy_; }

  /// Optimizes the computation of the parametric position p along the reference
  /// curve from the longitudinal position (in path-length) `s` along a parallel
  /// curve laterally offset by `r` from the reference curve.
  /// @return A function that relates longitudinal position `s` at the specified
  ///         parallel curve to parametric position p along the reference curve,
  ///         defined for all `s` values between 0 and the total path length of
  ///         the parallel curve (and throwing for any given value outside this
  ///         interval).
  /// @throws maliput::common::assertion_error When `r` makes the radius of
  ///         curvature be a non positive number.
  std::function<double(double)> OptimizeCalcPFromS(double r) const;

  /// Optimizes the computation of path length integral in the interval of the
  /// parameter [0; p] and along a parallel curve laterally offset by `r` the
  /// planar reference curve.
  /// @return A function that relates parametric position p along the reference
  ///         curve to longitudinal position s at the specified parallel curve,
  ///         defined for all p between 0 and 1 (and throwing for any given
  ///         value outside this interval).
  /// @throws maliput::common::assertion_error When `r` makes the radius of
  ///         curvature be a non positive number.
  std::function<double(double)> OptimizeCalcSFromP(double r) const;

  /// Computes the reference curve.
  /// @param p The reference curve parameter.
  /// @return The reference curve itself, F(p).
  virtual math::Vector2 xy_of_p(double p) const = 0;

  /// Computes the first derivative of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The derivative of the curve with respect to p, at @p p, i.e.,
  /// F'(p0) = (dx/dp, dy/dp) at p0.
  virtual math::Vector2 xy_dot_of_p(double p) const = 0;

  /// Computes the heading of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The heading of the curve at @p p, i.e.,
  /// the angle of the tangent vector (with respect to x-axis) in the
  /// increasing-p direction.
  virtual double heading_of_p(double p) const = 0;

  /// Computes the first derivative heading of the reference curve.
  /// @param p The reference curve parameter.
  /// @return The derivative of the heading with respect to p, i.e.,
  /// d_heading/dp evaluated at @p p.
  virtual double heading_dot_of_p(double p) const = 0;

  /// Computes the path length integral of the reference curve for
  /// the whole [0; 1] interval of p, formally l_max = ∫₀¹ |G'(p)| dp
  /// where G' = dG/dp.
  /// @return The total path length of the reference curve.
  virtual double l_max() const = 0;

  /// Converts a @p inertial_coordinate in the world frame to the composed curve
  /// frame, i.e., the superposition of the reference curve, elevation and
  /// superelevation polynomials. The resulting coordinates [p, r, h] are
  /// saturated in the following domain ranges.
  ///
  /// - p: [0, 1]
  /// - r: [@p r_min, @p r_max]
  /// - h: [@p height_bounds]
  ///
  /// @param inertial_coordinate A 3D vector in the world frame to be converted to
  /// the composed curve frame.
  /// @param r_min Minimum lateral distance from the composed curve to saturate,
  /// if it is necessary, the result in the given direction.
  /// @param r_max Maximum lateral distance from the composed curve to evaluate,
  /// if it is necessary, the result in the given direction
  /// @param height_bounds An api::HBounds object that represents the elevation
  /// bounds of the surface mapping.
  /// @return A 3D vector [p, r, h], that represent the domain coordinates of
  /// the world function, that gives as world function output @p inertial_coordinate.
  virtual math::Vector3 ToCurveFrame(const math::Vector3& inertial_coordinate, double r_min, double r_max,
                                     const api::HBounds& height_bounds) const = 0;

  /// Checks that there are no self-intersections (singularities) in the volume
  /// created by applying the constant @p r_min, @p r_max and @p height_bounds
  /// to the RoadCurve.
  /// @param r_min Minimum lateral distance from the composed curve to evaluate
  /// the validity of the geometry.
  /// @param r_max Maximum lateral distance from the composed curve to evaluate
  /// the validity of the geometry.
  /// @param height_bounds An api::HBounds object that represents the elevation
  /// bounds of the surface mapping.
  /// @return True when there are no self-intersections.
  virtual bool IsValid(double r_min, double r_max, const api::HBounds& height_bounds) const = 0;

  /// Returns W, the world function evaluated at @p p, @p r, @p h.
  math::Vector3 W_of_prh(double p, double r, double h) const;

  /// Returns W' = ∂W/∂p, the partial differential of W with respect to p,
  /// evaluated at @p p, @p r, @p h.
  ///
  /// (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  /// avoid recomputing it.)
  /// (@p g_prime must be the result of elevation().f_dot_p(p) --- passed in
  /// here to avoid recomputing it.)
  math::Vector3 W_prime_of_prh(double p, double r, double h, const Rot3& Rabg, double g_prime) const;

  /// Returns the rotation R_αβγ, evaluated at @p p along the reference curve.
  Rot3 Rabg_of_p(double p) const;

  /// Returns the rotation R_αβγ, evaluated at @p p, @p r and @p h.
  Rot3 Orientation(double p, double r, double h) const;

  /// Returns the s-axis unit-vector, expressed in the world frame,
  /// of the (s,r,h) `Lane`-frame (with respect to the world frame).
  ///
  /// (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  /// avoid recomputing it.)
  /// (@p g_prime must be the result of elevation().f_dot_p(p) --- passed in
  /// here to avoid recomputing it.)
  math::Vector3 s_hat_of_prh(double p, double r, double h, const Rot3& Rabg, double g_prime) const;

  /// Returns the r-axis unit-vector, expressed in the world frame,
  /// of the (s,r,h) `Lane`-frame (with respect to the world frame).
  ///
  /// (@p Rabg must be the result of Rabg_of_p(p) --- passed in here to
  /// avoid recomputing it.)
  math::Vector3 r_hat_of_Rabg(const Rot3& Rabg) const;

  /// Computes the most appropriate value for the elevation derivative g' at
  /// @p p, that accounts for the limitations of the arc length parameterization
  /// being used.
  /// @param p The reference curve parameter.
  /// @return The elevation derivative g'(@p p) value.
  /// @sa W_prime_of_prh()
  double CalcGPrimeAsUsedForCalcSFromP(double p) const;

 protected:
  /// Constructs a road curve given elevation and superelevation curves.
  /// @param linear_tolerance The linear tolerance, in meters, for all
  /// computations. It is understood in the the absolute error sense i.e.
  /// linear error must lie in the 0 ± linear tolerance interval, for
  /// @p scale_length long features at most.
  /// @param scale_length The minimum spatial period of variation in the
  /// curve, in meters. This imposes an upper limit to the spatial frequency
  /// (i.e. the Nyquist limit), which indicates the maximum level of detail
  /// expressed by the curve.
  /// @param elevation CubicPolynomial object that represents the elevation
  /// function (see below for more details).
  /// @param superelevation CubicPolynomial object that represents the
  /// superelevation function (see below for more details).
  /// @param computation_policy Policy to guide computations in terms of speed
  /// and accuracy. Actual behavior may vary across implementations.
  /// @pre The given @p scale_length is a positive number.
  /// @pre The given @p linear_tolerance is a positive number.
  /// @throws maliput::common::assertion_error if any of the preconditions is
  ///         not met.
  ///
  /// @p elevation and @p superelevation are cubic-polynomial functions which
  /// define the elevation and superelevation as a function of position along
  /// the planar reference curve.  @p elevation specifies the z-component of
  /// the surface at (r,h) = (0,0).  @p superelevation specifies the angle
  /// of the r-axis with respect to the horizon, i.e., how the road twists.
  /// Thus, non-zero @p superelevation contributes to the z-component at
  /// r != 0.
  ///
  /// These two functions (@p elevation and @p superelevation) must be
  /// isotropically scaled to operate over the domain p in [0, 1], where
  /// p is linear in the path-length of the planar reference curve,
  /// p = 0 corresponds to the start and p = 1 to the end. l_max()
  /// is the length of the reference curve. In other words...
  ///
  /// Given:
  ///  * a reference curve R(p) parameterized by p in domain [0, 1], which
  ///    has a path-length ℓ(p) in range [0, l_max], linearly related to p,
  ///    where l_max is the total path-length of R (in real-world units);
  ///  * the true elevation function E_true(ℓ), parameterized by the
  ///    path-length ℓ of R;
  ///  * the true superelevation function S_true(ℓ), parameterized by the
  ///    path-length ℓ of R;
  ///
  /// then:
  ///  * p = ℓ / l_max;
  ///  * @p elevation is E_scaled = (1 / l_max) * E_true(l_max * p);
  ///  * @p superelevation is  S_scaled = (1 / l_max) * S_true(l_max * p).
  RoadCurve(double linear_tolerance, double scale_length, const CubicPolynomial& elevation,
            const CubicPolynomial& superelevation, ComputationPolicy computation_policy);

 private:
  // Computes the minimum radius of curvature along a parallel curve at a
  // lateral distance @p r from the reference curve. Useful to identify
  // potentially ill-conditioned evaluations e.g. `r` offsets passing through
  // and past the instantaneous center of rotation.
  // @param r Lateral offset of the reference curve over the z = 0 plane.
  // @return The minimum radius of curvature.
  virtual double CalcMinimumRadiusAtOffset(double r) const = 0;

  // Computes the path length integral in the interval of the parameter [0; p]
  // and along a parallel curve laterally offset by `r` the planar reference
  // curve using numerical methods.
  // @return The path length integral of the curve composed with the elevation
  // polynomial.
  // @throws maliput::common::assertion_error When `r` makes the radius of
  //         curvature be a non positive number.
  double CalcSFromP(double p, double r) const;

  // TODO(hidmic): Fast, analytical methods and the conditions in which these
  // are expected to hold were tailored for the currently available
  // implementations. This limitation could be overcome by e.g. providing a
  // pair of virtual methods to check for the validity of the methods actually
  // implemented in the subclasses.

  // Resources fast, analytical methods to compute the parametric position
  // p along the reference curve corresponding to longitudinal position (in
  // path-length) `s` along a parallel curve laterally offset by `r` from the
  // reference curve.
  //
  // @remarks Said methods are only expected to be valid for curves that
  // show no superelevation and linear elevation at most.
  virtual double FastCalcPFromS(double s, double r) const = 0;

  // Resources fast, analytical methods to compute the path length integral
  // in the interval of the parameter [0; p] and along a parallel curve
  // laterally offset by `r` the planar reference curve.
  //
  // @remarks Said methods are only expected to be valid for curves that
  // show no superelevation and linear elevation at most.
  virtual double FastCalcSFromP(double p, double r) const = 0;

  // Checks whether fast, analytical methods would be accurate for the curve
  // as defined. It boils down to checking if the curve shows no superelevation
  // and linear elevation at most.
  //
  // @param r Lateral offset of the reference curve over the z=0 plane.
  // @return true if fast, analytical results would be accurate, and false
  // otherwise.
  bool AreFastComputationsAccurate(double r) const;

  // The minimum length of variations that the curve expresses.
  double scale_length_;
  // The tolerance for all computations, in the absolute error sense, for scale
  // length-long features in the road curve at most.
  double linear_tolerance_;
  // A polynomial that represents the elevation change as a function of p.
  CubicPolynomial elevation_;
  // A polynomial that represents the superelevation angle change as a function
  // of p.
  CubicPolynomial superelevation_;
  // A policy to guide computations in terms of speed and accuracy.
  ComputationPolicy computation_policy_;

  // Relative tolerance for numerical integrators.
  double relative_tolerance_;
  // The inverse arc length IVP, or the parameter p as a function of the
  // arc length s.
  std::unique_ptr<maliput::drake::systems::ScalarInitialValueProblem<double>> p_from_s_ivp_;
  // The arc length function, or the arc length s as a function of the
  // parameter p.
  std::unique_ptr<maliput::drake::systems::AntiderivativeFunction<double>> s_from_p_func_;
};

}  // namespace multilane
}  // namespace maliput
