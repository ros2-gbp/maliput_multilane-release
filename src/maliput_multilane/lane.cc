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
#include "maliput_multilane/lane.h"

#include <cmath>

#include <maliput/common/maliput_abort.h>
#include <maliput/math/vector.h>

#include "maliput_multilane/branch_point.h"

namespace maliput {
namespace multilane {

const api::Segment* Lane::do_segment() const { return segment_; }

const api::BranchPoint* Lane::DoGetBranchPoint(const api::LaneEnd::Which which_end) const {
  switch (which_end) {
    case api::LaneEnd::kStart: {
      return start_bp_;
    }
    case api::LaneEnd::kFinish: {
      return end_bp_;
    }
  }
  MALIPUT_ABORT_MESSAGE("which_end is neither LaneEnd::kStart nor LaneEnd::kFinish.");
}

const api::LaneEndSet* Lane::DoGetConfluentBranches(api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* Lane::DoGetOngoingBranches(api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetOngoingBranches({this, which_end});
}

std::optional<api::LaneEnd> Lane::DoGetDefaultBranch(api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetDefaultBranch({this, which_end});
}

api::InertialPosition Lane::DoToInertialPosition(const api::LanePosition& lane_pos) const {
  // Recover parameter p from arc-length position s.
  const double p = p_from_s_at_r0_(lane_pos.s());
  const math::Vector3 xyz = road_curve_->W_of_prh(p, lane_pos.r() + r0_, lane_pos.h());
  return {xyz.x(), xyz.y(), xyz.z()};
}

api::Rotation Lane::DoGetOrientation(const api::LanePosition& lane_pos) const {
  const double p = p_from_s_at_r0_(lane_pos.s());
  const Rot3 rotation = road_curve_->Orientation(p, lane_pos.r() + r0_, lane_pos.h());
  return api::Rotation::FromRpy(rotation.roll(), rotation.pitch(), rotation.yaw());
}

api::LanePosition Lane::DoEvalMotionDerivatives(const api::LanePosition& position,
                                                const api::IsoLaneVelocity& velocity) const {
  const double p = p_from_s_at_r0_(position.s());
  const double r = position.r() + r0_;
  const double h = position.h();
  const Rot3 R = road_curve_->Rabg_of_p(p);
  const double g_prime = road_curve_->elevation().f_dot_p(p);
  // Note that the elevation derivative value used to compute ds/dp may
  // not be the same as the one used for dσ/dp, to account for limitations
  // in s_from_p() computations.
  const double g_prime_for_ds = road_curve_->CalcGPrimeAsUsedForCalcSFromP(p);
  // The definition of path-length of a path along σ yields dσ = |∂W/∂p| dp
  // evaluated at (p, r, h).
  // Similarly, path-length s along the segment surface at r = r0 (which is
  // along the Lane's centerline) is related to p by ds = |∂W/∂p| dp evaluated
  // at (p, r0, 0).  Chaining yields ds/dσ:
  const double ds_dsigma = road_curve_->W_prime_of_prh(p, r0_, 0, R, g_prime_for_ds).norm() /
                           road_curve_->W_prime_of_prh(p, r, h, R, g_prime).norm();
  return api::LanePosition(ds_dsigma * velocity.sigma_v, velocity.rho_v, velocity.eta_v);
}

api::LanePositionResult Lane::DoToLanePosition(const api::InertialPosition& inertial_position) const {
  return InertialToLaneSegmentPositionBackend(inertial_position, true);
}

api::LanePositionResult Lane::DoToSegmentPosition(const api::InertialPosition& inertial_position) const {
  return InertialToLaneSegmentPositionBackend(inertial_position, false);
}

api::LanePositionResult Lane::InertialToLaneSegmentPositionBackend(const api::InertialPosition& inertial_position,
                                                                   bool use_lane_boundaries) const {
  // Computes the lateral extents of the surface in terms of the definition of
  // the reference curve. It implies a translation of the lane bounds
  // center by the lane by r0 distance.
  const double r_min = (use_lane_boundaries ? lane_bounds_.min() : segment_bounds_.min()) + r0_;
  const double r_max = (use_lane_boundaries ? lane_bounds_.max() : segment_bounds_.max()) + r0_;
  // Lane position is over the segment's road curve frame, so a change is
  // needed. That implies getting the path length s from p and translating the r
  // coordinate because of the offset.
  const math::Vector3 inertial_position_xyz{inertial_position.xyz()};
  const math::Vector3 lane_position_in_segment_curve_frame_drake =
      road_curve_->ToCurveFrame({inertial_position_xyz.x(), inertial_position_xyz.y(), inertial_position_xyz.z()},
                                r_min, r_max, elevation_bounds_);
  const math::Vector3 lane_position_in_segment_curve_frame{lane_position_in_segment_curve_frame_drake.x(),
                                                           lane_position_in_segment_curve_frame_drake.y(),
                                                           lane_position_in_segment_curve_frame_drake.z()};
  const double s = s_from_p_at_r0_(lane_position_in_segment_curve_frame[0]);
  const api::LanePosition lane_position =
      api::LanePosition(s, lane_position_in_segment_curve_frame[1] - r0_, lane_position_in_segment_curve_frame[2]);

  const api::InertialPosition nearest_position = ToInertialPosition(lane_position);

  return api::LanePositionResult{lane_position, nearest_position,
                                 (nearest_position.xyz() - inertial_position.xyz()).norm()};
}

}  // namespace multilane
}  // namespace maliput
