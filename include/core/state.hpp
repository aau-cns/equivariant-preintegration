// Copyright (C) 2024 Giulio Delama.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <giulio.delama@ieee.org>

#ifndef STATE_HPP
#define STATE_HPP

#include <groups/TG.hpp>

/**
 * @namespace preintegration
 * Namespace for the preintegration library components.
 */
namespace preintegration
{
  /**
   * @class PreintegrationState
   * @brief Represents the state containing preintegrated IMU measurements and extended bias.
   *
   * @tparam FPType. Floating point type (float, double, long double)
   */
  template <typename FPType>
  class PreintegrationState
  {
  public:
    using Vec10 = Eigen::Vector<FPType, 10>;
    using Mat5 = Eigen::Matrix<FPType, 5, 5>;
    using Gal3 = group::Gal3<FPType>;

    /**
     * @brief Default constructor initializing state with default values.
     */
    PreintegrationState() : Upsilon_(), bias_(Vec10::Zero()) {}

    /**
     * @brief Constructor initializing state with specific preintegrated IMU measurements and bias.
     * 
     * @param Upsilon Preintegrated IMU measurements in Gal3d format.
     * @param bias_j  Extended bias as a 10-dimensional vector.
     */
    PreintegrationState(const Gal3 &Upsilon, const Vec10 &bias_j)
        : Upsilon_(Upsilon), bias_(bias_j) {}

    /**
     * @brief Constructor initializing state with a matrix for preintegrated IMU measurements and specific bias.
     * 
     * @param Upsilon Preintegrated IMU measurements represented as a 5x5 matrix.
     * @param bias_j  Extended bias as a 10-dimensional vector.
     */
    PreintegrationState(const Mat5 &Upsilon, const Vec10 &bias_j)
        : Upsilon_(Upsilon), bias_(bias_j) {}

    /**
     * @brief Getter for the preintegrated IMU measurements.
     * 
     * @return A const reference to the preintegrated IMU measurements in Gal3d format.
     */
    const Gal3 &Upsilon() const { return Upsilon_; }

    /**
     * @brief Getter for the extended bias.
     * 
     * @return A const reference to the extended bias as a 10-dimensional vector.
     */
    const Vec10 &bias() const { return bias_; }

  private:
    /**
     * @brief Preintegrated IMU measurements in Gal3d format.
     */
    Gal3 Upsilon_;

    /**
     * @brief Extended bias represented as a 10-dimensional vector.
     */
    Vec10 bias_;
  };
} // namespace preintegration

#endif // STATE_HPP
