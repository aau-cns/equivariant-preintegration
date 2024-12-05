// Copyright (C) 2023 Giulio Delama.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <giulio.delama@ieee.org>

#ifndef TYPES_HPP
#define TYPES_HPP

#include <Eigen/Dense>
#include <TG.hpp>

namespace preintegration
{
  using Vec3 = Eigen::Vector<double, 3>;
  using Vec10 = Eigen::Vector<double, 10>;
  using Vec20 = Eigen::Vector<double, 20>;
  using Mat3 = Eigen::Matrix<double, 3, 3>;
  using Mat5 = Eigen::Matrix<double, 5, 5>;
  using Mat9 = Eigen::Matrix<double, 9, 9>;
  using Mat10 = Eigen::Matrix<double, 10, 10>;
  using Mat15 = Eigen::Matrix<double, 15, 15>;
  using Mat18 = Eigen::Matrix<double, 18, 18>;
  using Mat20 = Eigen::Matrix<double, 20, 20>;
  using Gal3 = group::Gal3<double>;
  using Gal3TG = group::Tangent<group::Gal3<double>>;
} // namespace preintegration

#endif // TYPES_HPP
