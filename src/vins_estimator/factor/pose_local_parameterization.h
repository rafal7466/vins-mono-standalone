#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

// Ceres >= 2.2 removed ceres::LocalParameterization. The pose parameterization of VINS-Mono (x = [p, q(x,y,z,w)],
// delta = [dp, dtheta], q+ = q * dq(dtheta)) is therefore expressed as a ceres::Manifold with the same Plus() and the
// same 7x6 Jacobian ([I6; 0]) as the original PoseLocalParameterization::ComputeJacobian().
class PoseLocalParameterization : public ceres::Manifold
{
  public:
    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    bool PlusJacobian(const double *x, double *jacobian) const override;
    bool Minus(const double *y, const double *x, double *y_minus_x) const override;
    bool MinusJacobian(const double *x, double *jacobian) const override;
};
