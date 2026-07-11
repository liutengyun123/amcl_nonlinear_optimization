// Copyright (c) 2018 Intel Corporation
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#ifndef NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
#define NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_

#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_vector.hpp"

namespace nav2_amcl
{

class MotionModel
{
public:
  virtual ~MotionModel() = default;

  virtual void initialize(
    double alpha1, double alpha2, double alpha3, double alpha4,
    double alpha5) = 0;

  virtual void odometryUpdate(
    pf_t * pf, const pf_vector_t & pose,
    const pf_vector_t & delta) = 0;
};

}  // namespace nav2_amcl

#endif  // NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
