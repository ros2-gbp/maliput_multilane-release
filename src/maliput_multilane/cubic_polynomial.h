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
#include <ostream>

#include <maliput/common/maliput_copyable.h>
#include <maliput/common/maliput_unused.h>

namespace maliput {
namespace multilane {

/// A cubic polynomial, f(p) = a + b*p + c*p^2 + d*p^3.
class CubicPolynomial {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CubicPolynomial)

  /// Default constructor, all zero coefficients.
  CubicPolynomial() : CubicPolynomial(0., 0., 0., 0.) {}

  /// Constructs a cubic polynomial given all four coefficients.
  CubicPolynomial(double a, double b, double c, double d) : a_(a), b_(b), c_(c), d_(d) {
    const double df = f_p(1.) - f_p(0.);
    s_1_ = std::sqrt(1. + (df * df));
  }

  // Returns the a coefficient.
  double a() const { return a_; }

  // Returns the b coefficient.
  double b() const { return b_; }

  // Returns the c coefficient.
  double c() const { return c_; }

  // Returns the d coefficient.
  double d() const { return d_; }

  /// Returns the order of the polynomial, based on
  /// its nonzero coefficients.
  int order() const {
    if (d_ != 0.0) return 3;
    if (c_ != 0.0) return 2;
    if (b_ != 0.0) return 1;
    return 0;
  }

  /// Checks whether the polynomial is _exactly_ zero.
  bool is_zero() const { return (a_ == 0.0 && b_ == 0.0 && c_ == 0.0 && d_ == 0.0); }

  /// Evaluates the polynomial f at @p p.
  double f_p(double p) const { return a_ + (b_ * p) + (c_ * p * p) + (d_ * p * p * p); }

  /// Evaluates the derivative df/dp at @p p.
  double f_dot_p(double p) const { return b_ + (2. * c_ * p) + (3. * d_ * p * p); }

  /// Evaluates the double-derivative d^2f/dp^2 at @p p.
  double f_ddot_p(double p) const { return (2. * c_) + (6. * d_ * p); }

  // TODO(maddog@tri.global)  s_p() and p_s() need to be replaced with a
  //                          properly integrated path-length parameterization.
  //                          For the moment, we are calculating the length by
  //                          approximating the curve with a single linear
  //                          segment from (0, f(0)) to (1, f(1)), which is
  //                          not entirely awful for relatively flat curves.
  /// Returns the path-length s along the curve (p, f(p)) from p = 0 to @p p.
  double s_p(double p) const { return s_1_ * p; }

  /// Returns the inverse of the path-length parameterization s_p(p).
  double p_s(double s) const { return s / s_1_; }

  // TODO(maddog@tri.global) Until s(p) is a properly integrated path-length
  //                         parameterization, we have a need to calculate the
  //                         derivative of the actual linear function
  //                         involved in our bogus path-length approximation.
  double fake_gprime(double p) const {
    maliput::common::unused(p);
    // return df;  which is...
    return f_p(1.) - f_p(0.);
  }

 private:
  double a_{};
  double b_{};
  double c_{};
  double d_{};
  double s_1_{};
};

/// Streams a string representation of `cubic_polynomial` into `out`.
/// Returns `out`. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const CubicPolynomial& cubic_polynomial);

}  // namespace multilane
}  // namespace maliput
