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

#ifndef INPUT_HPP
#define INPUT_HPP

#include <Eigen/Dense>

/**
 * @namespace preintegration
 * Namespace for the preintegration library components.
 */
namespace preintegration
{
  /**
   * @class PreintegrationInput
   * @brief Represents the input for preintegration, including IMU measurements and bias random walk.
   *
   * @tparam FPType. Floating point type (float, double, long double)
   */
  template <typename FPType>
  class PreintegrationInput
  {
  public:
    using Vec3 = Eigen::Vector<FPType, 3>;
    using Vec10 = Eigen::Vector<FPType, 10>;
    using Vec20 = Eigen::Vector<FPType, 20>;

    /**
     * @brief Default constructor initializing to zero.
     */
    PreintegrationInput() : w_(), tau_() {}

    /**
     * @brief Construct input from a 20-dimensional vector.
     *
     * @param u A 20-dimensional vector where the first 10 elements represent w_ and the last 10 represent tau_.
     */
    PreintegrationInput(const Vec20 &u) : w_(u.template head<10>()), tau_(u.template tail<10>()) {}

    /**
     * @brief Construct input from separate w and tau vectors.
     *
     * @param w   Preintegration of the IMU measurements.
     * @param tau Bias random walk input.
     */
    PreintegrationInput(const Vec10 &w, const Vec10 &tau)
        : w_(w), tau_(tau) {}

    /**
     * @brief Construct input from gyroscope and accelerometer measurements.
     *
     * @param gyroMeas Gyroscope measurements as a 3-dimensional vector.
     * @param accMeas  Accelerometer measurements as a 3-dimensional vector.
     */
    PreintegrationInput(const Vec3 &gyroMeas, const Vec3 &accMeas)
        : w_(), tau_()
    {
      w_ = Vec10::Zero();
      w_ << gyroMeas, accMeas, Vec3::Zero(), 1;
      tau_ = Vec10::Zero();
    }

    /**
     * @brief Get the preintegrated IMU measurements.
     *
     * @return A constant reference to the 10-dimensional vector representing w.
     */
    const Vec10 &w() const { return w_; }

    /**
     * @brief Get the bias random walk input.
     *
     * @return A constant reference to the 10-dimensional vector representing tau.
     */
    const Vec10 &tau() const { return tau_; }

    /**
     * @brief Get the gyroscope measurements from the input.
     *
     * @return A 3-dimensional vector representing gyroscope measurements.
     */
    const Vec3 omega() const { return w_.template head<3>(); }

    /**
     * @brief Get the accelerometer measurements from the input.
     *
     * @return A 3-dimensional vector representing accelerometer measurements.
     */
    const Vec3 acc() const { return w_.template segment<3>(3); }

    /**
     * @brief Get the full input vector combining w and tau.
     *
     * @return A 20-dimensional vector concatenating w and tau.
     */
    const Vec20 u() const { return (Vec20() << w_, tau_).finished(); }

  private:
    /**
     * @brief Preintegration of the IMU measurements.
     */
    Vec10 w_;

    /**
     * @brief Bias random walk input.
     */
    Vec10 tau_;
  };
} // namespace preintegration

#endif // INPUT_HPP
