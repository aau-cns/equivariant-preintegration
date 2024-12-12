# Equivariant Preintegration

[![License](https://img.shields.io/badge/License-AAUCNS-336B81.svg)](./LICENSE)
![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/aau-cns/Lie-plusplus/cmake-build-and-test.yml?label=Test)

Maintainer: [Giulio Delama](mailto:giulio.delama@aau.at)

- [Description](#description)
- [Installation](#installation)
- [Usage](#usage)
- [Credit](#credit)
  * [License](#license)
  * [Usage for academic purposes](#usage-for-academic-purposes)

## Description

This is a header-only C++ library based on Eigen for equivariant Inertial Measurement Unit (IMU) preintegration, a fundamental building
block that can be leveraged in different optimization-based Inertial Navigation System (INS) localization solutions.

## Installation

This is a header-only C++ library meant to be used within other projects. Either copy the content of `include` folder within the external project's `include` folder or use cmake's `FetchContent_Declare` as follows
```
FetchContent_Declare(
    EquivariantPreintegration
    GIT_REPOSITORY  https://github.com/aau-cns/equivariant-preintegration
    GIT_TAG         main
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
list(APPEND external EquivariantPreintegration) 
list(APPEND include_dirs ${EQUIVARIANT_PREINTEGRATION_INCLUDE_DIR})
list(APPEND libs EquivariantPreintegration Eigen3::Eigen)
```
## Usage
To use the Equivariant Preintegration library in your project, include the `preintegration.hpp` header from the `include` directory. Here is a sample usage in a test file:

```cpp
#include <iostream>
#include "core/preintegration.hpp"
#include "utils/tools.hpp"

int main() {
    using namespace preintegration;

    // Define the parameters
    std::shared_ptr<PreintegrationParams<double>> params = std::make_shared<PreintegrationParams<double>>();
    params->setGravity(Eigen::Vector3d::UnitZ() * -9.81);
    params->setGyroNoiseSigma(1e-4);
    params->setAccNoiseSigma(1e-3);
    params->setGyroBiasNoiseSigma(1e-6);
    params->setAccBiasNoiseSigma(1e-5);

    // Initialize the preintegration object
    EquivariantPreintegration<double> pim(params);

    // Example IMU measurements
    size_t n = 1000;
    double dt = 0.01;
    std::vector<Eigen::Vector3d> accs = utils::randomAcc<double>(-10, 10, n);
    std::vector<Eigen::Vector3d> gyros = utils::randomGyro<double>(-1, 1, n);

    // Integrate IMU measurements
    for (int j = 0; j < n; ++j) {
        pim.integrateMeasurement(accs[j], gyros[j], dt);
    }

    // Display preintegrated measurements
    std::cout << "Preintegration Matrix:\n"
              << pim.Upsilon().asMatrix() << std::endl;
    std::cout << "Preintegrated Rotation:\n"
              << pim.deltaRij() << std::endl;
    std::cout << "Preintegrated Velocity: " << pim.deltaVij().transpose() << std::endl;
    std::cout << "Preintegrated Position: " << pim.deltaPij().transpose() << std::endl;
    std::cout << "Preintegration Time: " << pim.deltaTij() << std::endl;
    std::cout << "Preintegration Covariance:\n" << pim.Cov() << "\n\n" << std::endl;

    return 0;
}
```

## Credit
This code was written within the [Control of Networked System (CNS)](https://www.aau.at/en/smart-systems-technologies/control-of-networked-systems/), University of Klagenfurt.

### License
This software is made available to the public to use (_source-available_), licensed under the terms of the BSD-2-Clause-License with no commercial use allowed, the full terms of which are made available in the `LICENSE` file. No license in patents is granted.

### Usage for academic purposes
If you use this software in an academic research setting, please cite the corresponding paper.

```latex
@ARTICLE{10777045,
  author={Delama, Giulio and Fornasier, Alessandro and Mahony, Robert and Weiss, Stephan},
  journal={IEEE Robotics and Automation Letters},
  title={Equivariant IMU Preintegration with Biases:a Galilean Group Approach},
  year={2024},
  pages={1-8},
  keywords={Lie groups;Navigation;Manifolds;Filtering theory;Vectors;Location awareness;Algebra;Accuracy;Libraries;Kalman filters;Localization;Sensor Fusion;SLAM},
  doi={10.1109/LRA.2024.3511424}}
```