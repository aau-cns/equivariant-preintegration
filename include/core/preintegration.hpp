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

#ifndef PREINTEGRATION_CORE_HPP
#define PREINTEGRATION_CORE_HPP

#include <memory>
#include "core/params.hpp"
#include "core/input.hpp"
#include "core/state.hpp"

/**
 * @namespace preintegration
 * Namespace for the preintegration library components.
 */
namespace preintegration
{
  /**
 * @class EquivariantPreintegration
 * @brief It is the main class for equivariant preintegration of IMU measurements [https://arxiv.org/pdf/2411.05548].
 *
 * @tparam FPType. Floating point type (float, double, long double)
 */
  template <typename FPType>
  class EquivariantPreintegration
  {
  public:
    using Scalar = FPType;
    using Vec3 = Eigen::Vector<FPType, 3>;
    using Vec10 = Eigen::Vector<FPType, 10>;
    using Vec20 = Eigen::Vector<FPType, 20>;
    using Mat3 = Eigen::Matrix<FPType, 3, 3>;
    using Mat5 = Eigen::Matrix<FPType, 5, 5>;
    using Mat9 = Eigen::Matrix<FPType, 9, 9>;
    using Mat10 = Eigen::Matrix<FPType, 10, 10>;
    using Mat15 = Eigen::Matrix<FPType, 15, 15>;
    using Mat18 = Eigen::Matrix<FPType, 18, 18>;
    using Mat20 = Eigen::Matrix<FPType, 20, 20>;
    using Gal3 = group::Gal3<FPType>;
    using Gal3TG = group::Tangent<group::Gal3<FPType>>;
    using State = PreintegrationState<FPType>;
    using Input = PreintegrationInput<FPType>;
    using Params = PreintegrationParams<FPType>;

    /**
    *  @brief Constructor, initializes the variables in the base class
    *  @param p    Parameters, typically fixed in a single application
    *  @param bias Current estimate of acceleration and rotation rate biases
    */
    EquivariantPreintegration(const std::shared_ptr<Params> &p,
                              const Vec10 &biasHat = Vec10::Zero(),
                              const State &xi0 = State())
        : p_(p), biasHat_(biasHat), xi0_(xi0)
    {
      resetIntegration();
    };

    // Re-initialize PreintegratedMeasurements
    void resetIntegration()
    {
      State xi_init = State(Gal3(), biasHat_);
      X_ = phi_inv(xi0_, xi_init);
      Jxi_ = Mat20::Identity();
      Cov_ = p_->getInitCov();
    }

    // Re-initialize PreintegratedMeasurements and set new bias
    void resetIntegrationAndSetBias(const Vec10 &biasHat)
    {
      biasHat_ = biasHat;
      resetIntegration();
    }

    // shared pointer to params
    const std::shared_ptr<Params> &params() const { return p_; }

    // const reference to params
    Params &p() const { return *p_; }

    // Accessors
    const Gal3TG &X() const { return X_; }
    const Vec10 &biasHat() const { return biasHat_; }
    const State &xi0() const { return xi0_; }
    const Mat20 &Cov() const { return Cov_; }
    const Mat20 &Jxi() const { return Jxi_; }

    // Derived accessors
    const State xi() const { return phi(X_, xi0_); }
    const Gal3 Upsilon() const { return xi().Upsilon(); }
    const Vec10 b() const { return xi().bias(); }
    const State xi_corrected(Vec10 bi) const
    {
      return State(Gal3::exp(Jb() * (bi - biasHat_)) * Upsilon(),
                   bi);
    }
    const Gal3 Upsilon_corrected(Vec10 bi) const
    {
      return Gal3::exp(Jb() * (bi - biasHat_)) * Upsilon();
    }

    const Mat3 deltaRij() const { return xi().Upsilon().R(); }
    const Vec3 deltaVij() const { return xi().Upsilon().v(); }
    const Vec3 deltaPij() const { return xi().Upsilon().p(); }
    const FPType deltaTij() const { return xi().Upsilon().s(); }

    const Mat10 Jb() const { return Jxi_.template topRightCorner<10, 10>(); }
    const Mat9 CovNav() const { return Cov_.template topLeftCorner<9, 9>(); }
    const Mat15 Cov15() const
    {
      Mat15 cov;
      cov.template block<9, 9>(0, 0) = CovNav();
      cov.template block<9, 6>(0, 9) = Cov_.template block<9, 6>(0, 10);
      cov.template block<6, 9>(9, 0) = Cov_.template block<6, 9>(10, 0);
      cov.template block<6, 6>(9, 9) = Cov_.template block<6, 6>(10, 10);
      return cov;
    }
    const Mat18 Cov18() const
    {
      Mat18 cov;
      cov.template block<9, 9>(0, 0) = CovNav();
      cov.template block<9, 9>(0, 9) = Cov_.template block<9, 9>(0, 10);
      cov.template block<9, 9>(9, 0) = Cov_.template block<9, 9>(10, 0);
      cov.template block<9, 9>(9, 9) = Cov_.template block<9, 9>(10, 10);
      return cov;
    }
    const Gal3 Gamma_ij() const
    {
      FPType dt = deltaTij();
      Mat5 Gamma = Mat5::Identity();
      Gamma.template block<3, 1>(0, 3) = p_->getGravity() * dt;
      Gamma.template block<3, 1>(0, 4) = -0.5 * p_->getGravity() * dt * dt;
      Gamma(3, 4) = -dt;
      return Gal3(Gamma);
    }
    const Gal3 invGamma_ij() const
    {
      FPType dt = deltaTij();
      Mat5 invGamma = Mat5::Identity();
      invGamma.template block<3, 1>(0, 3) = -p_->getGravity() * dt;
      invGamma.template block<3, 1>(0, 4) = -0.5 * p_->getGravity() * dt * dt;
      invGamma(3, 4) = dt;
      return Gal3(invGamma);
    }

    /**
     * @brief Applies the state action phi(X, xi).
     * @details Computes the action of a Tangent Group element X on a state element xi.
     * 
     * @param X A Tangent Group element (Gal3TG).
     * @param xi A state element on the manifold.
     * @return Resulting state after applying the action.
     */
    const State phi(const Gal3TG &X, const State &xi) const
    {
      return State(xi.Upsilon() * X.G(), X.G().inv().Adjoint() * (xi.bias() - X.g()));
    }

    /**
     * @brief Applies the inverse state action phi_inv(xi0, xi).
     * @details Computes the inverse action between two manifold elements xi0 and xi.
     * 
     * @param xi0 The initial state on the manifold.
     * @param xi The resulting state on the manifold.
     * @return Tangent Group element representing the inverse action.
     */
    const Gal3TG phi_inv(const State &xi0, const State &xi) const
    {
      Gal3 G = xi0.Upsilon().inv() * xi.Upsilon();
      Vec10 gamma = xi0.bias() - G.Adjoint() * xi.bias();
      return Gal3TG(G, gamma);
    };

    /**
     * @brief Applies the input action psi(X, u).
     * @details Computes the action of a Tangent Group element X on an input vector u in R^20.
     * 
     * @param X A Tangent Group element (Gal3TG).
     * @param u An input vector containing angular velocity and specific force (R^20).
     * @return Transformed input.
     */
    const Input psi(const Gal3TG &X, const Input &u) const
    {
      Vec10 w;
      Vec10 tau;
      Mat10 GinvAdj = X.G().inv().Adjoint();
      w << GinvAdj * (u.w() - X.g());
      tau << GinvAdj * u.tau();
      return Input(w, tau);
    }

    /**
     * @brief Computes the discrete-time lift Lambda(xi, u, dt).
     * @details Calculates the updated Tangent Group element after applying an input u over time dt.
     * 
     * @param xi The current state on the manifold.
     * @param u The input vector in R^20.
     * @param dt The time step duration.
     * @return The updated Tangent Group element.
     */
    const Gal3TG Lambda(const State &xi, const Input &u, FPType dt) const
    {
      Gal3 Lambda1 = Gal3::exp((u.w() - xi.bias()) * dt);
      Vec10 Lambda2 = xi.bias() - Lambda1.Adjoint() * (xi.bias() + u.tau() * dt);
      return Gal3TG(Lambda1, Lambda2);
    }

    /**
     * @brief Computes the logarithm of a Tangent Group element logTG(X).
     * @details Extracts the logarithmic coordinates of a Gal(3) Tangent Group element.
     * 
     * @param X A Tangent Group element (Gal3TG).
     * @return Logarithmic coordinates as a 20-dimensional vector.
     */
    const Vec20 logTG(const Gal3TG &X) const
    {
      Vec10 u1 = Gal3::log(X.G());
      Vec10 u2 = Gal3::invLeftJacobian(u1) * X.g();
      Vec20 u;
      u << u1, u2;
      return u;
    }

    /**
     * @brief Computes the normal coordinates of the Tangent Group theta(xi).
     * @details Maps a state element xi to its normal coordinates on the tangent space.
     * 
     * @param xi The state element on the manifold.
     * @return The normal coordinates as a 20-dimensional vector.
     */
    const Vec20 theta(const State &xi) const
    {
      return logTG(phi_inv(xi0_, xi));
    }

    /**
     * @brief Integrates an IMU measurement.
     * @details Updates the preintegrated state, covariance, and Jacobians using accelerometer
     * and gyroscope measurements over a time step.
     * 
     * @param accMeas Measured specific force (accelerometer reading).
     * @param gyroMeas Measured angular velocity (gyroscope reading).
     * @param dt Time step duration.
     */
    void integrateMeasurement(const Vec3 &accMeas, const Vec3 &gyroMeas, const FPType dt)
    {
      Input u(gyroMeas, accMeas);
      Mat10 K =
          xi().Upsilon().Adjoint() * Gal3::leftJacobian((u.w() - b()) * dt) * dt;
      Input u0 = psi(X_.inv(), u);

      // Propagate mean
      X_ = X_ * Lambda(phi(X_, xi0_), u, dt);

      // Propagate covariance
      Mat20 A = Mat20::Identity();
      A.template topRightCorner<10, 10>() = Gal3::leftJacobian(u0.w() * dt) * dt;
      A.template bottomRightCorner<10, 10>() = Gal3::exp(u0.w() * dt).Adjoint();

      Mat20 B = Mat20::Zero();
      B.template topLeftCorner<10, 10>() = K;
      B.template bottomRightCorner<10, 10>() = -xi().Upsilon().Adjoint() * dt;

      Cov_ = A * Cov_ * A.transpose() + B * (p_->Qc() / dt) * B.transpose();

      // Propagate state Jacobian wrt bias
      Mat20 Phi_b = Mat20::Identity();
      Phi_b.template topRightCorner<10, 10>() = -K;

      Jxi_ = Phi_b * Jxi_;
    }

  private:
    std::shared_ptr<Params> p_;

    // Initial bias estimate used for preintegration
    Vec10 biasHat_;

    // State origin (Identity element)
    State xi0_;

    // Preintegration from i to j as a Tangent Group element
    Gal3TG X_;

    // Jacobian of the state with respect to the bias (used for bias update)
    Mat20 Jxi_;

    // Covariance matrix for the preintegrated measurements
    Mat20 Cov_;
  };

} // namespace preintegration

#endif // PREINTEGRATION_CORE_HPP
