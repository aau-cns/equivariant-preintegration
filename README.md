# Equivariant Preintegration

[![License](https://img.shields.io/badge/License-AAUCNS-336B81.svg)](./LICENSE)
![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/aau-cns/Lie-plusplus/cmake-build-and-test.yml?label=Test)

Maintainer: [Giulio Delama](mailto:giulio.delama@aau.at)

- [Description](#description)
- [Dependencies](#dependencies)
- [Installation](#installation)
  * [Run Tests](#tests)
- [Usage](#usage)
- [Credit](#credit)
  * [License](#license)
  * [Usage for academic purposes](#usage-for-academic-purposes)

## Description

This is a header-only C++ library for equivariant Inertial Measurement Unit (IMU) preintegration, a fundamental building
block that can be leveraged in different optimization-based Inertial Navigation System (INS) localization solutions.
By utilizing an equivariant symmetry, our method geometrically couples navigation states and biases, leading to reduced errors and improved consistency compared to existing state-of-the-art techniques ([paper](https://ieeexplore.ieee.org/document/10777045)).

## Dependencies

The library has the following dependencies which are automatically downloaded and linked against:

- [Eigen](https://gitlab.com/libeigen/eigen.git)
- [Lie++](https://github.com/aau-cns/Lie-plusplus)
- [googletest](https://github.com/google/googletest.git)

## Installation

This is a header-only C++ library meant to be used within other projects. Either copy the content of `include` folder within the external project's `include` folder or use cmake's `FetchContent_Declare` as follows
```cmake
FetchContent_Declare(
    EquivariantPreintegration
    GIT_REPOSITORY  https://github.com/aau-cns/equivariant-preintegration
    GIT_TAG         main
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
list(APPEND external EquivariantPreintegration) 
list(APPEND include_dirs ${EQUIVARIANTPREINTEGRATION_INCLUDE_DIR})
list(APPEND libs EquivariantPreintegration)
```
Alternatively, if you want to clone the repository and do a local build, follow these steps:
```sh
git clone https://github.com/aau-cns/equivariant-preintegration.git
cd equivariant-preintegration
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=True
cmake --build .
```

### Tests
If you have built the library with `-DBUILD_TESTS=True`, you can execute the unit tests by running the following command inside the build directory:
```sh
./equivariant_preintegration_tests
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
  title={Equivariant IMU Preintegration With Biases: A Galilean Group Approach}, 
  year={2025},
  volume={10},
  number={1},
  pages={724-731},
  keywords={Lie groups;Navigation;Manifolds;Filtering theory;Vectors;Location awareness;Algebra;Accuracy;Libraries;Kalman filters;Localization;sensor fusion;SLAM},
  doi={10.1109/LRA.2024.3511424}}
```
