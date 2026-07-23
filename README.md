# High-fidelity Project Chrono-based Simulator for UAVs

[![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)

## 📚 Overview

This codebase is a C++ implementation of Project Chrono that provides a high-fidelity physics engine to perform model-in-the-loop, software-in-the-loop, and hardware-in-the-loop simulations with the freeware, open-source, PX4-compatible [**ACSL Flightstack Winged**](https://github.com/andrealaffly/ACSL-flightstack-winged). The codebase leverages the Chrono simulation environment to test various **GNC** (Guidance, Navigation and Control) algorithms prior to real-world flights, enabling debugging, optimization, and tuning of algorithms formulated at the Advanced Control Systems Lab (ACSL) at Virginia Tech.

### System Requirements

- **RAM:** At least 16 GB
- **GPU:** Nvidia
- **Operating System:** Ubuntu 20.04 or above, Windows 11 WSL2.

#### Example - Ideal Setup

| **Component**             | **Details**                                |
|---------------------------|--------------------------------------------|
| **CPU**                   | AMD Ryzen 5 9600X                          |
| **GPU**                   | Nvidia RTX 3080 (10 GB)                    |
| **Operating System**      | Ubuntu 24.04                               |
| **RAM**                   | 32 GB                                      |

## Documentation

For additional details, see our [Wiki](https://github.com/andrealaffly/acsl-chrono-simulator/wiki).

## Maintenance Team

- [**Andrea L'Afflitto**](https://github.com/andrealaffly)
- [**Giri Mugundan Kumar**](https://github.com/girimugundankumar)

[![ACSL Flight Stack Logo](https://lafflitto.com/images/ACSL_Logo.jpg)](https://lafflitto.com/ACSL.html)

## 📝 License

This software is distributed under a permissive BSD 3-Clause License. See the [![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE) file for details. The Chrono Project is licensed under the BSD 3-Clause License. See the [![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/projectchrono/chrono/blob/9.0.1/LICENSE) file for more details.