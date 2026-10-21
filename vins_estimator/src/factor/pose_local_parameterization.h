/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/version.h>
#include "../utility/utility.h"

// Ceres 2.1 introduced ceres::Manifold and 2.2 removed
// ceres::LocalParameterization; the pose parameterization is a Manifold from 2.1
// on, with the same Plus and Plus-Jacobian.
#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
#define VINS_CERES_MANIFOLD 1
#endif

#ifdef VINS_CERES_MANIFOLD
class PoseLocalParameterization : public ceres::Manifold
{
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    bool PlusJacobian(const double *x, double *jacobian) const override;
    bool Minus(const double *y, const double *x, double *y_minus_x) const override;
    bool MinusJacobian(const double *x, double *jacobian) const override;
    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }
};
#else
class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};
#endif
