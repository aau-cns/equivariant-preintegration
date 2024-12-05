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

#include <utils/types.hpp>

/**
 * @namespace preintegration
 * Namespace for the preintegration library components.
 */
namespace preintegration
{

  /**
   * @class Params
   * @brief Represents configuration parameters for the system.
   */
  class Params
  {
  private:
    /**
     * @brief Gravity vector.
     */
    Vec3 gravity_;

    /**
     * @brief Continuous time gyroscope noise sigma.
     */
    double gyroNoiseSigma_;

    /**
     * @brief Continuous time accelerometer noise sigma.
     */
    double accNoiseSigma_;

    /**
     * @brief Continuous time gyroscope bias random walk sigma.
     */
    double gyroBiasNoiseSigma_;

    /**
     * @brief Continuous time accelerometer bias random walk sigma.
     */
    double accBiasNoiseSigma_;

    /**
     * @brief Initial state covariance matrix.
     */
    Mat20 initCov_;

  public:
    /**
     * @brief Constructor initializing the parameters with default or specified values.
     *
     * @param gravity Gravity vector (default: -9.81 in the Z direction).
     * @param gyroNoiseSigma Gyroscope noise sigma (default: 1e-4).
     * @param accNoiseSigma Accelerometer noise sigma (default: 1e-3).
     * @param gyroBiasNoiseSigma Gyroscope bias noise sigma (default: 1e-6).
     * @param accBiasNoiseSigma Accelerometer bias noise sigma (default: 3e-5).
     * @param initCov Initial state covariance matrix (default: zero matrix).
     */
    Params(Vec3 gravity = Vec3::UnitZ() * -9.81,
           double gyroNoiseSigma = 1e-4,
           double accNoiseSigma = 1e-3,
           double gyroBiasNoiseSigma = 1e-6,
           double accBiasNoiseSigma = 3e-5,
           Mat20 initCov = Mat20::Zero())
        : gravity_(gravity),
          gyroNoiseSigma_(gyroNoiseSigma),
          accNoiseSigma_(accNoiseSigma),
          gyroBiasNoiseSigma_(gyroBiasNoiseSigma),
          accBiasNoiseSigma_(accBiasNoiseSigma),
          initCov_(initCov) {}

    /**
     * @brief Destructor.
     */
    ~Params() {}

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
    double getGyroNoiseSigma() const { return gyroNoiseSigma_; }

    /**
     * @brief Set the gyroscope noise sigma.
     *
     * @param sigma Gyroscope noise sigma to set.
     */
    void setGyroNoiseSigma(double sigma) { gyroNoiseSigma_ = sigma; }

    /**
     * @brief Get the accelerometer noise sigma.
     *
     * @return Accelerometer noise sigma.
     */
    double getAccNoiseSigma() const { return accNoiseSigma_; }

    /**
     * @brief Set the accelerometer noise sigma.
     *
     * @param sigma Accelerometer noise sigma to set.
     */
    void setAccNoiseSigma(double sigma) { accNoiseSigma_ = sigma; }

    /**
     * @brief Get the gyroscope bias noise sigma.
     *
     * @return Gyroscope bias noise sigma.
     */
    double getGyroBiasNoiseSigma() const { return gyroBiasNoiseSigma_; }

    /**
     * @brief Set the gyroscope bias noise sigma.
     *
     * @param sigma Gyroscope bias noise sigma to set.
     */
    void setGyroBiasNoiseSigma(double sigma) { gyroBiasNoiseSigma_ = sigma; }

    /**
     * @brief Get the accelerometer bias noise sigma.
     *
     * @return Accelerometer bias noise sigma.
     */
    double getAccBiasNoiseSigma() const { return accBiasNoiseSigma_; }

    /**
     * @brief Set the accelerometer bias noise sigma.
     *
     * @param sigma Accelerometer bias noise sigma to set.
     */
    void setAccBiasNoiseSigma(double sigma) { accBiasNoiseSigma_ = sigma; }

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
      Mat20 Qc = Mat20::Zero();
      Qc.topLeftCorner<3, 3>().diagonal().setConstant(gyroNoiseSigma_ * gyroNoiseSigma_);
      Qc.block<3, 3>(3, 3).diagonal().setConstant(accNoiseSigma_ * accNoiseSigma_);
      Qc.block<3, 3>(10, 10).diagonal().setConstant(gyroBiasNoiseSigma_ * gyroBiasNoiseSigma_);
      Qc.block<3, 3>(13, 13).diagonal().setConstant(accBiasNoiseSigma_ * accBiasNoiseSigma_);
      return Qc;
    }
  };

} // namespace preintegration

#endif // PARAMS_HPP
