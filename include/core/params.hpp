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

#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <Eigen/Dense>

/**
 * @namespace preintegration
 * Namespace for the preintegration library components.
 */
namespace preintegration
{

  /**
   * @class PreintegrationParams
   * @brief Represents configuration parameters for the system.
   *
   * @tparam FPType. Floating point type (float, double, long double)
   */
  template <typename FPType>
  class PreintegrationParams
  {
  public:
    using Vec3 = Eigen::Vector<FPType, 3>;
    using Mat3 = Eigen::Matrix<FPType, 3, 3>;
    using Mat20 = Eigen::Matrix<FPType, 20, 20>;

    /**
     * @brief Constructor initializing the parameters with default or specified values.
     *
     * @param gravity Gravity vector (default: -9.81 in the Z direction).
     * @param gyroNoiseSigma Gyroscope noise sigma (default: 1).
     * @param accNoiseSigma Accelerometer noise sigma (default: 1).
     * @param gyroBiasNoiseSigma Gyroscope bias noise sigma (default: 1).
     * @param accBiasNoiseSigma Accelerometer bias noise sigma (default: 1).
     * @param initCov Initial state covariance matrix (default: zero matrix).
     */
    PreintegrationParams(Vec3 gravity = Vec3::UnitZ() * -9.81,
                         FPType gyroNoiseSigma = 1,
                         FPType accNoiseSigma = 1,
                         FPType gyroBiasNoiseSigma = 1,
                         FPType accBiasNoiseSigma = 1,
                         Mat20 initCov = Mat20::Zero())
        : gravity_(gravity),
          gyroNoiseSigma_(gyroNoiseSigma),
          accNoiseSigma_(accNoiseSigma),
          gyroBiasNoiseSigma_(gyroBiasNoiseSigma),
          accBiasNoiseSigma_(accBiasNoiseSigma),
          initCov_(initCov) {}

    /**
     * @brief Get the gravity vector.
     *
     * @return Gravity vector.
     */
    Vec3 getGravity() const { return gravity_; }

    /**
     * @brief Set the gravity vector.
     *
     * @param gravity Gravity vector to set.
     */
    void setGravity(const Vec3 &gravity) { gravity_ = gravity; }

    /**
     * @brief Get the gyroscope noise sigma.
     *
     * @return Gyroscope noise sigma.
     */
    FPType getGyroNoiseSigma() const { return gyroNoiseSigma_; }

    /**
     * @brief Set the gyroscope noise sigma.
     *
     * @param sigma Gyroscope noise sigma to set.
     */
    void setGyroNoiseSigma(FPType sigma) { gyroNoiseSigma_ = sigma; }

    /**
     * @brief Get the accelerometer noise sigma.
     *
     * @return Accelerometer noise sigma.
     */
    FPType getAccNoiseSigma() const { return accNoiseSigma_; }

    /**
     * @brief Set the accelerometer noise sigma.
     *
     * @param sigma Accelerometer noise sigma to set.
     */
    void setAccNoiseSigma(FPType sigma) { accNoiseSigma_ = sigma; }

    /**
     * @brief Get the gyroscope bias noise sigma.
     *
     * @return Gyroscope bias noise sigma.
     */
    FPType getGyroBiasNoiseSigma() const { return gyroBiasNoiseSigma_; }

    /**
     * @brief Set the gyroscope bias noise sigma.
     *
     * @param sigma Gyroscope bias noise sigma to set.
     */
    void setGyroBiasNoiseSigma(FPType sigma) { gyroBiasNoiseSigma_ = sigma; }

    /**
     * @brief Get the accelerometer bias noise sigma.
     *
     * @return Accelerometer bias noise sigma.
     */
    FPType getAccBiasNoiseSigma() const { return accBiasNoiseSigma_; }

    /**
     * @brief Set the accelerometer bias noise sigma.
     *
     * @param sigma Accelerometer bias noise sigma to set.
     */
    void setAccBiasNoiseSigma(FPType sigma) { accBiasNoiseSigma_ = sigma; }

    /**
     * @brief Get the initial state covariance matrix.
     *
     * @return Initial state covariance matrix.
     */
    Mat20 getInitCov() const { return initCov_; }

    /**
     * @brief Set the initial state covariance matrix.
     *
     * @param initCov Initial state covariance matrix to set.
     */
    void setInitCov(const Mat20 &initCov) { initCov_ = initCov; }

    /**
     * @brief Compute the process noise covariance matrix.
     *
     * @return Process noise covariance matrix.
     */
    Mat20 Qc() const
    {
      Mat20 Qc = Mat20::Identity();
      Qc.template block<3, 3>(0, 0) *= gyroNoiseSigma_ * gyroNoiseSigma_;
      Qc.template block<3, 3>(3, 3) *= accNoiseSigma_ * accNoiseSigma_;
      Qc.template block<4, 4>(6, 6) *= 0;
      Qc.template block<3, 3>(10, 10) *= gyroBiasNoiseSigma_ * gyroBiasNoiseSigma_;
      Qc.template block<3, 3>(13, 13) *= accBiasNoiseSigma_ * accBiasNoiseSigma_;
      Qc.template block<4, 4>(16, 16) *= 0;
      return Qc;
    }

  private:
    /**
     * @brief Gravity vector.
     */
    Vec3 gravity_;

    /**
     * @brief Continuous time gyroscope noise sigma.
     */
    FPType gyroNoiseSigma_;

    /**
     * @brief Continuous time accelerometer noise sigma.
     */
    FPType accNoiseSigma_;

    /**
     * @brief Continuous time gyroscope bias random walk sigma.
     */
    FPType gyroBiasNoiseSigma_;

    /**
     * @brief Continuous time accelerometer bias random walk sigma.
     */
    FPType accBiasNoiseSigma_;

    /**
     * @brief Initial state covariance matrix.
     */
    Mat20 initCov_;
  };

} // namespace preintegration

#endif // PARAMS_HPP
