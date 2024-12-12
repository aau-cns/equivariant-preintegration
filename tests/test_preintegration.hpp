// Copyright (C) 2024 Giulio Delama.
// Control of Networked Systems, University of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the authors at <giulio.delama@ieee.org>.

#ifndef TEST_PREINTEGRATION_HPP
#define TEST_PREINTEGRATION_HPP

#include <unsupported/Eigen/MatrixFunctions>
#include "core/preintegration.hpp"
#include "utils/tools.hpp"

namespace test
{
  using namespace preintegration;

  typedef testing::Types<EquivariantPreintegration<float>, EquivariantPreintegration<double>> PreintegrationTypes;

  /**
   * @brief Equivariant Preintegration specific tests
   */
  template <typename T>
  class PreintegrationTest : public testing::Test
  {
  };
  TYPED_TEST_SUITE(PreintegrationTest, PreintegrationTypes);

  TYPED_TEST(PreintegrationTest, Constructors)
  {
    for (int i = 0; i < N_TESTS; ++i)
    {
      {
        std::shared_ptr<typename TypeParam::Params> p = std::make_shared<typename TypeParam::Params>();
        TypeParam pim = TypeParam(p);
        typename TypeParam::Mat20 Qc = TypeParam::Mat20::Identity();
        Qc.template block<4, 4>(6, 6) *= 0;
        Qc.template block<4, 4>(16, 16) *= 0;
        MatrixEquality(pim.Cov(), TypeParam::Mat20::Zero());
        MatrixEquality(pim.Jxi(), TypeParam::Mat20::Identity());
        MatrixEquality(pim.X().G().asMatrix(), typename TypeParam::Gal3().asMatrix());
        MatrixEquality(pim.X().g(), TypeParam::Vec10::Zero());
        MatrixEquality(pim.biasHat(), TypeParam::Vec10::Zero());
        MatrixEquality(pim.xi0().Upsilon().asMatrix(), typename TypeParam::Gal3().asMatrix());
        MatrixEquality(pim.xi0().bias(), TypeParam::Vec10::Zero());
        MatrixEquality(pim.xi().Upsilon().asMatrix(), typename TypeParam::Gal3().asMatrix());
        MatrixEquality(pim.xi().bias(), TypeParam::Vec10::Zero());
        MatrixEquality(pim.Upsilon().asMatrix(), typename TypeParam::Gal3().asMatrix());
        MatrixEquality(pim.b(), TypeParam::Vec10::Zero());
        MatrixEquality(pim.CovNav(), TypeParam::Mat9::Zero());
        MatrixEquality(pim.Cov15(), TypeParam::Mat15::Zero());
        MatrixEquality(pim.Cov18(), TypeParam::Mat18::Zero());
        MatrixEquality(pim.deltaRij(), TypeParam::Mat3::Identity());
        MatrixEquality(pim.deltaVij(), TypeParam::Vec3::Zero());
        MatrixEquality(pim.deltaPij(), TypeParam::Vec3::Zero());
        ScalarEquality(pim.deltaTij(), 0);
        MatrixEquality(pim.Gamma_ij().asMatrix(), TypeParam::Mat5::Identity());
        MatrixEquality(pim.invGamma_ij().asMatrix(), TypeParam::Mat5::Identity());
        MatrixEquality(pim.Jb(), TypeParam::Mat10::Zero());
        MatrixEquality(pim.p().getGravity(), TypeParam::Vec3::UnitZ() * -9.81);
        ScalarEquality(pim.p().getGyroNoiseSigma(), 1);
        ScalarEquality(pim.p().getAccNoiseSigma(), 1);
        ScalarEquality(pim.p().getGyroBiasNoiseSigma(), 1);
        ScalarEquality(pim.p().getAccBiasNoiseSigma(), 1);
        MatrixEquality(pim.p().getInitCov(), TypeParam::Mat20::Zero());
        MatrixEquality(pim.p().Qc(), Qc);
      }
      {
        std::shared_ptr<typename TypeParam::Params> p = std::make_shared<typename TypeParam::Params>(TypeParam::Vec3::UnitZ() * -9.81, 1e-4, 1e-3, 1e-6, 1e-5);
        TypeParam pim = TypeParam(p);
        typename TypeParam::Mat20 Qc = TypeParam::Mat20::Identity();
        Qc.template block<3, 3>(0, 0) *= 1e-4 * 1e-4;
        Qc.template block<3, 3>(3, 3) *= 1e-3 * 1e-3;
        Qc.template block<4, 4>(6, 6) *= 0;
        Qc.template block<3, 3>(10, 10) *= 1e-6 * 1e-6;
        Qc.template block<3, 3>(13, 13) *= 1e-5 * 1e-5;
        Qc.template block<4, 4>(16, 16) *= 0;
        MatrixEquality(pim.p().getGravity(), TypeParam::Vec3::UnitZ() * -9.81);
        ScalarEquality(pim.p().getGyroNoiseSigma(), 1e-4);
        ScalarEquality(pim.p().getAccNoiseSigma(), 1e-3);
        ScalarEquality(pim.p().getGyroBiasNoiseSigma(), 1e-6);
        ScalarEquality(pim.p().getAccBiasNoiseSigma(), 1e-5);
        MatrixEquality(pim.p().getInitCov(), TypeParam::Mat20::Zero());
        MatrixEquality(pim.p().Qc(), Qc);
      }
    }
  }

  TYPED_TEST(PreintegrationTest, MeanPropagation)
  {
    for (int i = 0; i < N_TESTS; ++i)
    {
      size_t n = 100;
      typename TypeParam::Scalar dt = 0.01;
      std::vector<typename TypeParam::Vec3> accs = utils::randomAcc<typename TypeParam::Scalar>(-10, 10, n);
      std::vector<typename TypeParam::Vec3> gyros = utils::randomGyro<typename TypeParam::Scalar>(-1, 1, n);
      std::shared_ptr<typename TypeParam::Params> p = std::make_shared<typename TypeParam::Params>(TypeParam::Vec3::UnitZ() * -9.81, 1e-4, 1e-3, 1e-6, 1e-5);
      TypeParam pim = TypeParam(p);
      typename TypeParam::Mat3 Delta_Rij = TypeParam::Mat3::Identity();
      typename TypeParam::Vec3 Delta_Vij = TypeParam::Vec3::Zero();
      typename TypeParam::Vec3 Delta_Pij = TypeParam::Vec3::Zero();
      typename TypeParam::Scalar Delta_Tij = 0;
      for (int j = 0; j < n; ++j)
      {
        pim.integrateMeasurement(accs[j], gyros[j], dt);
        Delta_Pij += Delta_Vij * dt + Delta_Rij * group::SO3<typename TypeParam::Scalar>::Gamma2(gyros[j] * dt) * accs[j] * dt * dt;
        Delta_Vij += Delta_Rij * group::SO3<typename TypeParam::Scalar>::leftJacobian(gyros[j] * dt) * accs[j] * dt;
        Delta_Rij *= group::SO3<typename TypeParam::Scalar>::exp(gyros[j] * dt).asMatrix();
        Delta_Tij += dt;
      }
      MatrixEquality(pim.deltaRij(), Delta_Rij);
      MatrixEquality(pim.deltaVij(), Delta_Vij);
      MatrixEquality(pim.deltaPij(), Delta_Pij);
      ScalarEquality(pim.deltaTij(), Delta_Tij);
    }
  }

  TYPED_TEST(PreintegrationTest, CovariancePropagation)
  {
    for (int i = 0; i < N_TESTS; ++i)
    {
      size_t n = 100;
      typename TypeParam::Scalar dt = 0.01;
      std::vector<typename TypeParam::Vec3> accs = utils::randomAcc<typename TypeParam::Scalar>(-10, 10, n);
      std::vector<typename TypeParam::Vec3> gyros = utils::randomGyro<typename TypeParam::Scalar>(-1, 1, n);
      std::shared_ptr<typename TypeParam::Params> p = std::make_shared<typename TypeParam::Params>(TypeParam::Vec3::UnitZ() * -9.81, 1e-4, 1e-3, 1e-6, 1e-5);
      TypeParam pim = TypeParam(p);
      group::Gal3<typename TypeParam::Scalar> Upsilon = group::Gal3<typename TypeParam::Scalar>();
      typename TypeParam::Mat20 A = TypeParam::Mat20::Identity();
      typename TypeParam::Mat20 B = TypeParam::Mat20::Zero();
      typename TypeParam::Mat20 Cov = pim.Cov();
      typename TypeParam::Mat20 Phi_bias = TypeParam::Mat20::Identity();
      typename TypeParam::Mat20 Jxi = TypeParam::Mat20::Identity();
      for (int j = 0; j < n; ++j)
      {
        pim.integrateMeasurement(accs[j], gyros[j], dt);
        typename TypeParam::Vec10 w, w0;
        w << gyros[j], accs[j], 0, 0, 0, 1;
        w0 = Upsilon.Adjoint() * (w - pim.biasHat());
        A.template topRightCorner<10, 10>() = group::Gal3<typename TypeParam::Scalar>::leftJacobian(w0 * dt) * dt;
        A.template bottomRightCorner<10, 10>() = group::Gal3<typename TypeParam::Scalar>::exp(w0 * dt).Adjoint();
        B.template topLeftCorner<10, 10>() = Upsilon.Adjoint() * group::Gal3<typename TypeParam::Scalar>::leftJacobian((w - pim.biasHat()) * dt) * dt;
        Upsilon = Upsilon * group::Gal3<typename TypeParam::Scalar>::exp(w * dt);
        B.template bottomRightCorner<10, 10>() = -Upsilon.Adjoint() * dt;
        Cov = A * Cov * A.transpose() + B * (p->Qc() / dt) * B.transpose();
        Phi_bias.template topRightCorner<10, 10>() = -B.template topLeftCorner<10, 10>();
        Jxi = Phi_bias * Jxi;
      }
      MatrixEquality(pim.Cov(), Cov);
      MatrixEquality(pim.Jxi(), Jxi);
    }
  }

} // namespace test

#endif // TEST_PREINTEGRATION_HPP