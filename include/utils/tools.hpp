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

#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <queue>
#include <random>
#include <type_traits>
#include <Eigen/Dense>

namespace utils
{
  /**
 * @brief Generate random numbers
 *
 * @tparam Numeric type of random number generated
 * @tparam Generator random number generator
 * @param from Lower bound
 * @param to Upper bound
 * @return Numeric
 *
 * @note Modified from:
 * https://stackoverflow.com/questions/2704521/generate-random-double-numbers-in-c
 */
  template <typename Numeric, typename Generator = std::mt19937>
  static Numeric random(Numeric from, Numeric to)
  {
    thread_local static Generator gen(std::random_device{}());
    using dist_type = typename std::conditional<std::is_integral<Numeric>::value, std::uniform_int_distribution<Numeric>,
                                                std::uniform_real_distribution<Numeric>>::type;
    thread_local static dist_type dist;
    return dist(gen, typename dist_type::param_type{from, to});
  }

  /**
 * @brief Generate random IMU acceleration measurements
 * 
 * @tparam FPType Floating point type
 * @param from Lower bound
 * @param to Upper bound
 * @param n Number of measurements
 * 
 * @return std::vector<Eigen::Vector3<FPType>>
 */
  template <typename FPType>
  static std::vector<Eigen::Vector3<FPType>> randomAcc(const FPType from, const FPType to, const size_t n)
  {
    std::vector<Eigen::Vector3<FPType>> accs;
    for (size_t i = 0; i < n; ++i)
    {
      accs.push_back(Eigen::Vector3<FPType>(random(from, to), random(from, to), random(from, to)));
    }
    return accs;
  }

  /**
 * @brief Generate random IMU gyro measurements
 * 
 * @tparam FPType Floating point type
 * @param from Lower bound
 * @param to Upper bound
 * @param n Number of measurements
 * 
 * @return std::vector<Eigen::Vector3<FPType>>
 */
  template <typename FPType>
  static std::vector<Eigen::Vector3<FPType>> randomGyro(const FPType from, const FPType to, const size_t n)
  {
    std::vector<Eigen::Vector3<FPType>> gyros;
    for (size_t i = 0; i < n; ++i)
    {
      gyros.push_back(Eigen::Vector3<FPType>(random(from, to), random(from, to), random(from, to)));
    }
    return gyros;
  }

} // namespace utils

#endif // TOOLS_HPP
