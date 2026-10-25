// Relative-pose factor with exact Jacobians.
//
// Residual e = Log(Z^-1 * Xi^-1 * Xj) under right perturbation X (+) xi = X * Exp(xi), xi = (omega, v).
// With h = Z^-1 Xi^-1 Xj (so e = Log(h)):
//   Xj -> Xj Exp(xi):  h -> h Exp(xi)                                  =>  de/dxi_j =  Jr^-1(e)
//   Xi -> Xi Exp(xi):  Xi^-1 Xj -> Xi^-1 Xj Exp(-Ad(Xj^-1 Xi) xi)      =>  de/dxi_i = -Jr^-1(e) Ad(Xj^-1 Xi)
//   left-multiplying by the constant Z^-1 adds no Jacobian term.
// gtsam::BetweenFactor drops Jr^-1(e) unless GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR is defined,
// which is only accurate for small residuals; a fresh loop edge has the largest residual of all.
// Checked against numerical differences in test_pose_graph.
#pragma once

#include <boost/make_shared.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace uslam_backend
{

class RelativePoseFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>
{
public:
  RelativePoseFactor(gtsam::Key i, gtsam::Key j, const gtsam::Pose3 & z, const gtsam::SharedNoiseModel & model)
  : gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3>(model, i, j), z_inv_(z.inverse()) {}

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & xi, const gtsam::Pose3 & xj,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override
  {
    gtsam::Matrix66 d_rel_xi, d_rel_xj, d_e_h;
    const gtsam::Pose3 rel = xi.between(xj, d_rel_xi, d_rel_xj);  // d_rel_xi = -Ad(Xj^-1 Xi), d_rel_xj = I
    const gtsam::Vector6 e = gtsam::Pose3::Logmap(z_inv_ * rel, d_e_h);  // d_e_h = Jr^-1(e)
    if (H1) {*H1 = d_e_h * d_rel_xi;}
    if (H2) {*H2 = d_e_h * d_rel_xj;}
    return e;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::make_shared<RelativePoseFactor>(*this);
  }

private:
  gtsam::Pose3 z_inv_;
};

}  // namespace uslam_backend
